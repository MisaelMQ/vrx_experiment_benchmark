#!/usr/bin/env python3

import math
import os
from typing import Optional, Dict, Any, List

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Float64MultiArray

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from ament_index_python.packages import get_package_share_directory

from vrx_experiment_benchmark.route_utils import (
    load_route_yaml,
    compute_local_route_from_latlon,
    distance_xy,
    MODE_TO_CODE,
)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


# State machine tracking trajectory sequence and spatial progression
class RouteManager(Node):
    def __init__(self) -> None:
        super().__init__("route_manager")

        self.declare_parameter("route_file", "")
        self.declare_parameter("origin_topic", "/wamv/navigation/local_origin")
        self.declare_parameter("state_topic", "/wamv/navigation/state2d")
        self.declare_parameter("route_local_topic", "/wamv/navigation/route_local")
        self.declare_parameter("active_waypoint_topic", "/wamv/navigation/active_waypoint")
        self.declare_parameter("active_waypoint_meta_topic", "/wamv/navigation/active_waypoint_meta")
        self.declare_parameter("route_status_topic", "/wamv/navigation/route_status")

        self.declare_parameter("frame_id", "map_local")
        self.declare_parameter("start_hold_sec", 10.0)
        self.declare_parameter("finish_hold_sec", 15.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("debug_enabled", True)
        self.declare_parameter("debug_interval_sec", 1.0)

        self.route_file = str(self.get_parameter("route_file").value)
        self.origin_topic = str(self.get_parameter("origin_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.route_local_topic = str(self.get_parameter("route_local_topic").value)
        self.active_waypoint_topic = str(self.get_parameter("active_waypoint_topic").value)
        self.active_waypoint_meta_topic = str(self.get_parameter("active_waypoint_meta_topic").value)
        self.route_status_topic = str(self.get_parameter("route_status_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.start_hold_sec = float(self.get_parameter("start_hold_sec").value)
        self.finish_hold_sec = float(self.get_parameter("finish_hold_sec").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.debug_enabled = bool(self.get_parameter("debug_enabled").value)
        self.debug_interval_sec = float(self.get_parameter("debug_interval_sec").value)

        if not self.route_file:
            raise RuntimeError("Parameter 'route_file' is required")

        self.route_file = self.resolve_route_file(self.route_file)
        self.route_raw: Dict[str, Any] = load_route_yaml(self.route_file)
        self.route_local: Optional[Dict[str, Any]] = None

        self.origin_lat: Optional[float] = None
        self.origin_lon: Optional[float] = None

        self.state_msg: Optional[Odometry] = None
        self.active_idx: int = 0
        self.route_ready: bool = False
        self.route_completed: bool = False

        self.hold_started: bool = False
        self.hold_start_time_sec: Optional[float] = None
        self.last_report_time_sec: float = 0.0
        self.last_announced_wp_id: Optional[int] = None
        self.was_inside_last_cycle: bool = False

        origin_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        default_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Float64MultiArray, self.origin_topic, self.origin_callback, origin_qos)
        self.create_subscription(Odometry, self.state_topic, self.state_callback, default_qos)

        self.route_pub = self.create_publisher(Path, self.route_local_topic, default_qos)
        self.active_wp_pub = self.create_publisher(PoseStamped, self.active_waypoint_topic, default_qos)
        self.active_wp_meta_pub = self.create_publisher(Float64MultiArray, self.active_waypoint_meta_topic, default_qos)
        self.route_status_pub = self.create_publisher(Float64MultiArray, self.route_status_topic, default_qos)

        self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self.timer_callback)

        self.get_logger().info(f"Loaded route file: {self.route_file}")

    def resolve_route_file(self, route_file: str) -> str:
        if os.path.isabs(route_file) and os.path.exists(route_file):
            return route_file

        pkg_share = get_package_share_directory("vrx_experiment_benchmark")
        candidates = [
            route_file,
            os.path.join(pkg_share, route_file),
            os.path.join(pkg_share, "config", route_file),
            os.path.join(pkg_share, "config", "routes", route_file),
        ]

        for candidate in candidates:
            if os.path.exists(candidate):
                return os.path.abspath(candidate)

        raise RuntimeError(
            "Could not resolve route_file='{}'. Tried: {}".format(
                route_file, ", ".join(candidates)
            )
        )

    # Wait for dynamic map origin configuration
    def origin_callback(self, msg: Float64MultiArray) -> None:
        if self.route_ready:
            return

        if len(msg.data) < 2:
            self.get_logger().warn("Origin message has insufficient data")
            return

        self.origin_lat = float(msg.data[0])
        self.origin_lon = float(msg.data[1])

        self.route_local = compute_local_route_from_latlon(
            self.route_raw,
            self.origin_lat,
            self.origin_lon,
        )

        if len(self.route_local["waypoints"]) == 0:
            raise RuntimeError("Converted route has zero waypoints")

        self.route_ready = True
        self.active_idx = 0
        self.hold_started = False
        self.hold_start_time_sec = None
        self.last_announced_wp_id = None
        self.was_inside_last_cycle = False

        self.get_logger().info(
            f"Route converted to local XY using origin lat={self.origin_lat:.10f}, lon={self.origin_lon:.10f}"
        )
        self.log_route_summary()
        self.publish_route_path()

    # Cache current Odometry state
    def state_callback(self, msg: Odometry) -> None:
        self.state_msg = msg

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def get_current_waypoint(self) -> Optional[Dict[str, Any]]:
        if not self.route_ready or self.route_local is None:
            return None
        if self.active_idx >= len(self.route_local["waypoints"]):
            return None
        return self.route_local["waypoints"][self.active_idx]

    def get_segment_start_end(self) -> Optional[List[float]]:
        if not self.route_ready or self.route_local is None:
            return None

        waypoints = self.route_local["waypoints"]
        if self.active_idx >= len(waypoints):
            return None

        curr = waypoints[self.active_idx]

        if self.active_idx == 0:
            x0 = curr["x"]
            y0 = curr["y"]
            if len(waypoints) > 1:
                x1 = waypoints[1]["x"]
                y1 = waypoints[1]["y"]
            else:
                x1 = curr["x"]
                y1 = curr["y"]
        else:
            prev = waypoints[self.active_idx - 1]
            x0 = prev["x"]
            y0 = prev["y"]
            x1 = curr["x"]
            y1 = curr["y"]

        return [x0, y0, x1, y1]

    def distance_to_current_waypoint(self) -> Optional[float]:
        wp = self.get_current_waypoint()
        if wp is None or self.state_msg is None:
            return None

        x = self.state_msg.pose.pose.position.x
        y = self.state_msg.pose.pose.position.y
        return distance_xy(x, y, wp["x"], wp["y"])

    def required_hold_time(self, mode: str) -> float:
        if mode == "start":
            return self.start_hold_sec
        if mode == "finish":
            return self.finish_hold_sec
        return 0.0

    def log_route_summary(self) -> None:
        if self.route_local is None:
            return

        waypoints = self.route_local["waypoints"]
        total_length = 0.0
        for i in range(1, len(waypoints)):
            total_length += distance_xy(
                waypoints[i - 1]["x"],
                waypoints[i - 1]["y"],
                waypoints[i]["x"],
                waypoints[i]["y"],
            )

        self.get_logger().info(
            "Route summary: id=%s waypoints=%d approx_length=%.2f m" % (
                self.route_local.get("route_id", "unknown_route"),
                len(waypoints),
                total_length,
            )
        )
        for wp in waypoints:
            self.get_logger().info(
                "  WP id=%s mode=%s tol=%.2f xy=(%.2f, %.2f) path_yaw=%.3f rad" % (
                    wp.get("id", "?"),
                    wp.get("mode", "transit"),
                    float(wp.get("pos_tolerance", 3.0)),
                    float(wp["x"]),
                    float(wp["y"]),
                    float(wp.get("path_yaw", 0.0)),
                )
            )

    # Verify and announce target update in stdout
    def announce_active_waypoint(self) -> None:
        wp = self.get_current_waypoint()
        if wp is None:
            return

        wp_id = int(wp.get("id", self.active_idx))
        if self.last_announced_wp_id == wp_id:
            return

        seg = self.get_segment_start_end()
        if seg is None:
            return

        seg_len = distance_xy(seg[0], seg[1], seg[2], seg[3])
        self.get_logger().info(
            "Active waypoint -> idx=%d id=%d mode=%s tol=%.2f xy=(%.2f, %.2f) seg_start=(%.2f, %.2f) seg_end=(%.2f, %.2f) seg_len=%.2f m" % (
                self.active_idx,
                wp_id,
                str(wp.get("mode", "transit")),
                float(wp.get("pos_tolerance", 3.0)),
                float(wp["x"]),
                float(wp["y"]),
                float(seg[0]),
                float(seg[1]),
                float(seg[2]),
                float(seg[3]),
                seg_len,
            )
        )
        self.last_announced_wp_id = wp_id

    # Determine stage progression through position tolerances and holds
    def update_progress(self) -> None:
        if not self.route_ready or self.route_completed or self.state_msg is None:
            return

        wp = self.get_current_waypoint()
        if wp is None:
            return

        dist = self.distance_to_current_waypoint()
        if dist is None:
            return

        tolerance = float(wp.get("pos_tolerance", 3.0))
        mode = str(wp.get("mode", "transit"))
        hold_required = self.required_hold_time(mode)

        inside = dist <= tolerance
        wp_id = int(wp.get("id", self.active_idx))

        if inside and not self.was_inside_last_cycle:
            self.get_logger().info(
                "Entered waypoint tolerance: idx=%d id=%d mode=%s dist=%.2f tol=%.2f" % (
                    self.active_idx,
                    wp_id,
                    mode,
                    dist,
                    tolerance,
                )
            )
        elif (not inside) and self.was_inside_last_cycle:
            self.get_logger().info(
                "Exited waypoint tolerance: idx=%d id=%d mode=%s dist=%.2f tol=%.2f" % (
                    self.active_idx,
                    wp_id,
                    mode,
                    dist,
                    tolerance,
                )
            )
        self.was_inside_last_cycle = inside

        if mode == "transit":
            # Geometric projection on the segment to detect overshoot
            seg = self.get_segment_start_end()
            proj = 0.0
            if seg is not None:
                px = self.state_msg.pose.pose.position.x
                py = self.state_msg.pose.pose.position.y
                sdx = seg[2] - seg[0]
                sdy = seg[3] - seg[1]
                seg_len2 = sdx * sdx + sdy * sdy
                if seg_len2 > 1e-6:
                    proj = ((px - seg[0]) * sdx + (py - seg[1]) * sdy) / seg_len2

            # Criteria 1: entered position tolerance. Criteria 2: overshot waypoint via segment projection.
            passed = proj >= 1.05
            should_advance = inside or passed

            if should_advance:
                reason = "inside_tolerance" if inside else "overshoot"
                self.get_logger().info(
                    "\n╔════════════════════════════════════════════╗\n"
                    "║  ▶ WP %d → WP %d  [%s]  dist=%.1fm  proj=%.2f  (%s)  ║\n"
                    "╚════════════════════════════════════════════╝" % (
                        wp_id, wp_id + 1, mode.upper(), dist, proj, reason,
                    )
                )
                self.active_idx += 1
                self.hold_started = False
                self.hold_start_time_sec = None
                self.last_announced_wp_id = None
                self.was_inside_last_cycle = False

        elif mode in ("start", "finish"):
            if inside:
                if not self.hold_started:
                    self.hold_started = True
                    self.hold_start_time_sec = self.now_sec()
                    self.get_logger().info(
                        "\n┌────────────────────────────────────────┐\n"
                        "│  ⏸ HOLD  idx=%d  mode=%s  wait=%.1fs  │\n"
                        "└────────────────────────────────────────┘" % (
                            self.active_idx, mode.upper(), hold_required,
                        )
                    )

                elapsed = self.now_sec() - float(self.hold_start_time_sec)
                if elapsed >= hold_required:
                    if mode == "finish":
                        self.route_completed = True
                        self.get_logger().info(
                            "\n██████████████████████████████████████████████\n"
                            "  ★ ROUTE COMPLETED  hold=%.1fs  WP=%d  ★\n"
                            "██████████████████████████████████████████████" % (
                                elapsed, wp_id,
                            )
                        )
                    else:
                        self.get_logger().info(
                            "\n╔════════════════════════════════════════════╗\n"
                            "║  ✅ HOLD OK  idx=%d  hold=%.1fs  → advancing ║\n"
                            "╚════════════════════════════════════════════╝" % (
                                self.active_idx, elapsed,
                            )
                        )
                        self.active_idx += 1
                        self.last_announced_wp_id = None
                    self.hold_started = False
                    self.hold_start_time_sec = None
                    self.was_inside_last_cycle = False
            else:
                if self.hold_started:
                    self.get_logger().warn(
                        "Hold reset at waypoint idx=%d id=%d because vehicle left tolerance zone" % (
                            self.active_idx,
                            wp_id,
                        )
                    )
                self.hold_started = False
                self.hold_start_time_sec = None

        # Fallback: if active_idx exceeds all waypoints without a finish waypoint hold,
        # mark as completed. Note: this should not happen in properly defined routes.
        if self.route_local is not None and self.active_idx >= len(self.route_local["waypoints"]):
            self.route_completed = True
            self.get_logger().warn("All waypoints processed via index overflow -> route completed (check route definition)")

    # Log physical progress along tracking line
    def maybe_log_periodic_status(self) -> None:
        if not self.debug_enabled or self.state_msg is None or not self.route_ready:
            return

        now = self.now_sec()
        if (now - self.last_report_time_sec) < max(self.debug_interval_sec, 0.05):
            return
        self.last_report_time_sec = now

        wp = self.get_current_waypoint()
        if wp is None:
            if self.route_completed:
                self.get_logger().info("Periodic status: route already completed")
            return

        dist = self.distance_to_current_waypoint()
        if dist is None:
            return

        x = self.state_msg.pose.pose.position.x
        y = self.state_msg.pose.pose.position.y
        tolerance = float(wp.get("pos_tolerance", 3.0))
        mode = str(wp.get("mode", "transit"))
        hold_elapsed = 0.0
        if self.hold_started and self.hold_start_time_sec is not None:
            hold_elapsed = self.now_sec() - self.hold_start_time_sec

        # Scalar projection of the vehicle on the active segment
        seg = self.get_segment_start_end()
        proj = float("nan")
        if seg is not None:
            sdx = seg[2] - seg[0]
            sdy = seg[3] - seg[1]
            seg_len2 = sdx * sdx + sdy * sdy
            if seg_len2 > 1e-6:
                proj = ((x - seg[0]) * sdx + (y - seg[1]) * sdy) / seg_len2

        self.get_logger().info(
            "RT[%d/%d(w:%s):%s]|v:(%.1f,%.1f)w:(%.1f,%.1f)|d:%.1f(t:%.1f)|pj:%.2f|h:%.1f/%.1f|dn:%s" % (
                self.active_idx,
                len(self.route_local["waypoints"]) - 1 if self.route_local else -1,
                str(wp.get("id", self.active_idx)), mode[:3], x, y, float(wp["x"]), float(wp["y"]),
                dist, tolerance, proj, hold_elapsed, self.required_hold_time(mode), str(self.route_completed)[0]
            )
        )

    # Yield static representation of visual path
    def publish_route_path(self) -> None:
        if not self.route_ready or self.route_local is None:
            return

        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        for wp in self.route_local["waypoints"]:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(wp["x"])
            ps.pose.position.y = float(wp["y"])
            ps.pose.position.z = 0.0
            ps.pose.orientation = yaw_to_quaternion(float(wp.get("path_yaw", 0.0)))
            msg.poses.append(ps)

        self.route_pub.publish(msg)

    # Push current target pose
    def publish_active_waypoint(self) -> None:
        wp = self.get_current_waypoint()
        if wp is None:
            return

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.frame_id
        ps.pose.position.x = float(wp["x"])
        ps.pose.position.y = float(wp["y"])
        ps.pose.position.z = 0.0
        ps.pose.orientation = yaw_to_quaternion(float(wp.get("path_yaw", 0.0)))
        self.active_wp_pub.publish(ps)

    # Sync complex logic metadata downstream to trackers
    def publish_active_waypoint_meta(self) -> None:
        wp = self.get_current_waypoint()
        seg = self.get_segment_start_end()
        if wp is None or seg is None:
            return

        mode = str(wp.get("mode", "transit"))
        hold_required = self.required_hold_time(mode)
        hold_elapsed = 0.0
        if self.hold_started and self.hold_start_time_sec is not None:
            hold_elapsed = self.now_sec() - self.hold_start_time_sec

        msg = Float64MultiArray()
        msg.data = [
            float(wp.get("id", self.active_idx)),
            float(MODE_TO_CODE.get(mode, 1)),
            float(wp.get("pos_tolerance", 3.0)),
            float(wp["x"]),
            float(wp["y"]),
            float(wp.get("path_yaw", 0.0)),
            float(seg[0]),
            float(seg[1]),
            float(seg[2]),
            float(seg[3]),
            float(self.active_idx),
            float(len(self.route_local["waypoints"])) if self.route_local else 0.0,
            float(hold_elapsed),
            float(hold_required),
            1.0 if self.route_completed else 0.0,
        ]
        self.active_wp_meta_pub.publish(msg)

    # Announce general lifecycle status
    def publish_route_status(self) -> None:
        dist = self.distance_to_current_waypoint()
        if dist is None:
            dist = float("nan")

        msg = Float64MultiArray()
        msg.data = [
            float(self.active_idx),
            float(len(self.route_local["waypoints"])) if self.route_local else 0.0,
            float(dist),
            1.0 if self.route_ready else 0.0,
            1.0 if self.route_completed else 0.0,
        ]
        self.route_status_pub.publish(msg)

    # Core execution and topic update rate timer
    def timer_callback(self) -> None:
        if not self.route_ready:
            return

        self.announce_active_waypoint()
        self.update_progress()
        self.publish_route_path()
        # Publish metadata ALWAYS (even when completed) so guidance_los
        # receives route_completed=True and halts guidance_cmd.
        if not self.route_completed:
            self.publish_active_waypoint()
            self.publish_active_waypoint_meta()
        else:
            self.publish_active_waypoint_meta()  # final ping [14]=1.0
        self.publish_route_status()
        self.maybe_log_periodic_status()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()