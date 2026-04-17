#!/usr/bin/env python3
"""guidance_los — Line of Sight (LOS) guidance node for the WAM-V.

# Public Interface (Compatible with MPPI or other controllers)
# ============================================================
# To replace this node, the new node must:
#
#   Required Subscriptions:
#     /wamv/navigation/state2d            → nav_msgs/Odometry
#     /wamv/navigation/active_waypoint_meta → std_msgs/Float64MultiArray (15 fields)
#
#   Required Publication:
#     /wamv/control/guidance_cmd → std_msgs/Float64MultiArray (12 fields)
#         [0] psi_ref        desired heading [rad]
#         [1] u_ref          desired surge speed [m/s]
#         [2] e_ct           cross-track error [m]
#         [3] chi_p          segment path yaw [rad]
#         [4] dist_to_wp     distance to active waypoint [m]
#         [5] delta          lookahead distance [m]
#         [6] heading_error  psi_ref - yaw [rad]
#         [7] wp_id          waypoint ID [float]
#         [8] mode_code      0=start, 1=transit, 2=finish
#         [9] pos_tol        position tolerance [m]
#         [10] wp_x          waypoint X-coordinate [m]
#         [11] wp_y          waypoint Y-coordinate [m]
#
#   mode_codes: 0="start", 1="transit", 2="finish"
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from vrx_experiment_benchmark.route_utils import wrap_angle, quaternion_to_yaw, distance_xy, CODE_TO_MODE


# Line of sight guidance algorithm implementation
class LOSGuidance(Node):
    def __init__(self) -> None:
        super().__init__("guidance_los")

        self.declare_parameter("state_topic", "/wamv/navigation/state2d")
        self.declare_parameter("active_waypoint_meta_topic", "/wamv/navigation/active_waypoint_meta")
        self.declare_parameter("guidance_topic", "/wamv/control/guidance_cmd")
        self.declare_parameter("publish_rate_hz", 20.0)

        self.declare_parameter("lookahead_min", 4.0)
        self.declare_parameter("lookahead_max", 12.0)
        self.declare_parameter("lookahead_speed_gain", 2.5)

        self.declare_parameter("start_speed", 2.0)
        self.declare_parameter("transit_speed", 4.5)
        self.declare_parameter("finish_speed", 1.5)
        self.declare_parameter("slowdown_radius", 4.0)

        self.declare_parameter("debug_enabled", True)
        self.declare_parameter("debug_interval_sec", 1.0)

        self.state_topic = str(self.get_parameter("state_topic").value)
        self.active_waypoint_meta_topic = str(self.get_parameter("active_waypoint_meta_topic").value)
        self.guidance_topic = str(self.get_parameter("guidance_topic").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.lookahead_min = float(self.get_parameter("lookahead_min").value)
        self.lookahead_max = float(self.get_parameter("lookahead_max").value)
        self.lookahead_speed_gain = float(self.get_parameter("lookahead_speed_gain").value)

        self.start_speed = float(self.get_parameter("start_speed").value)
        self.transit_speed = float(self.get_parameter("transit_speed").value)
        self.finish_speed = float(self.get_parameter("finish_speed").value)
        self.slowdown_radius = float(self.get_parameter("slowdown_radius").value)

        self.debug_enabled = bool(self.get_parameter("debug_enabled").value)
        self.debug_interval_sec = float(self.get_parameter("debug_interval_sec").value)

        self.state_msg: Optional[Odometry] = None
        self.meta_msg: Optional[Float64MultiArray] = None
        self.last_report_time_sec: float = 0.0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Odometry, self.state_topic, self.state_callback, qos)
        self.create_subscription(Float64MultiArray, self.active_waypoint_meta_topic, self.meta_callback, qos)

        self.guidance_pub = self.create_publisher(Float64MultiArray, self.guidance_topic, qos)

        self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self.timer_callback)

        self.get_logger().info("guidance_los started")

    # Update vehicle state
    def state_callback(self, msg: Odometry) -> None:
        self.state_msg = msg

    # Update active waypoint metadata
    def meta_callback(self, msg: Float64MultiArray) -> None:
        self.meta_msg = msg

    # Get ROS time in seconds
    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # Speed-dependent dynamic lookahead distance calculation
    def compute_lookahead(self, speed: float) -> float:
        value = self.lookahead_min + self.lookahead_speed_gain * abs(speed)
        return max(self.lookahead_min, min(self.lookahead_max, value))

    def nominal_speed(self, mode: str, distance_to_wp: float, pos_tol: float,
                       seg_len: float = 999.0) -> float:
        # Reference speed with adaptive deceleration near waypoint
        stop_radius = max(pos_tol, 1.0)

        if mode == "start":
            if distance_to_wp <= stop_radius:
                return 0.0
            if distance_to_wp > 10.0:
                return self.start_speed
            scale = max(0.45, min(1.0, distance_to_wp / 10.0))
            return max(0.5, self.start_speed * scale)

        if mode == "finish":
            if distance_to_wp <= stop_radius:
                return 0.0
            scale = max(0.35, min(1.0, distance_to_wp / max(self.slowdown_radius, 1e-6)))
            return max(0.3, self.finish_speed * scale)

        # Transit mode: cap speed on short segments to capture WP
        seg_speed_cap = self.transit_speed
        if seg_len < 20.0:
            # Scale: 10m -> 2.0 m/s, >20m -> full transit_speed
            seg_speed_cap = max(2.0, self.transit_speed * (seg_len / 20.0))

        # Smooth deceleration when approaching waypoint
        if distance_to_wp < self.slowdown_radius:
            scale = max(0.4, min(1.0, distance_to_wp / max(self.slowdown_radius, 1e-6)))
            return max(1.5, seg_speed_cap * scale)

        return seg_speed_cap

    # Periodically log guidance performance
    def maybe_log_debug(
        self,
        wp_id: int,
        mode: str,
        x: float,
        y: float,
        yaw: float,
        speed: float,
        wp_x: float,
        wp_y: float,
        chi_p: float,
        e_ct: float,
        delta: float,
        psi_ref: float,
        heading_error: float,
        u_ref: float,
        distance_to_wp: float,
        proj: float = 0.0,
    ) -> None:
        if not self.debug_enabled:
            return

        now = self.now_sec()
        if (now - self.last_report_time_sec) < max(self.debug_interval_sec, 0.05):
            return
        self.last_report_time_sec = now

        self.get_logger().info(
            "LOS[wp:%d:%s]|p:(%.1f,%.1f)-w:(%.1f,%.1f)|yw:%.2f,u:%.2f|chi:%.2f,ect:%.2f,d:%.2f|ref:(%.2f,%.1f)|e_h:%.2f|dw:%.1f,prj:%.2f" % (
                wp_id, mode[:3], x, y, wp_x, wp_y, yaw, speed, chi_p, e_ct, delta, psi_ref, u_ref, heading_error, distance_to_wp, proj
            )
        )

    # Compute heading and speed references continuously
    def timer_callback(self) -> None:
        if self.state_msg is None or self.meta_msg is None:
            return

        data = self.meta_msg.data
        if len(data) < 15:
            return

        wp_id = int(data[0])
        mode_code = int(data[1])
        pos_tol = float(data[2])
        wp_x = float(data[3])
        wp_y = float(data[4])
        # data[5] = path_yaw (no usado directamente)
        seg_x0 = float(data[6])
        seg_y0 = float(data[7])
        seg_x1 = float(data[8])
        seg_y1 = float(data[9])
        route_completed = bool(int(data[14]))

        if route_completed:
            # Publish stop command allowing controllers to halt thrusters
            if self.state_msg is not None:
                q = self.state_msg.pose.pose.orientation
                yaw_now = quaternion_to_yaw(q.x, q.y, q.z, q.w)
                stop_msg = Float64MultiArray()
                stop_msg.data = [
                    float(yaw_now),  # psi_ref = current yaw -> error_yaw = 0
                    0.0,             # u_ref = 0 -> no forward speed
                    0.0, 0.0, 0.0, 0.0, 0.0,  # e_ct, chi_p, dist, delta, e_h
                    float(data[0]), float(data[1]), float(data[2]),  # wp_id, mode, tol
                    float(data[3]), float(data[4]),                  # wp_x, wp_y
                ]
                self.guidance_pub.publish(stop_msg)
            return

        mode = CODE_TO_MODE.get(mode_code, "transit")

        x = self.state_msg.pose.pose.position.x
        y = self.state_msg.pose.pose.position.y
        vx = self.state_msg.twist.twist.linear.x
        vy = self.state_msg.twist.twist.linear.y
        speed = math.hypot(vx, vy)

        q = self.state_msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        distance_to_wp = distance_xy(x, y, wp_x, wp_y)

        seg_dx = seg_x1 - seg_x0
        seg_dy = seg_y1 - seg_y0
        chi_p = math.atan2(seg_dy, seg_dx)

        e_ct = -math.sin(chi_p) * (x - seg_x0) + math.cos(chi_p) * (y - seg_y0)

        delta = self.compute_lookahead(speed)

        # Scalar projection [0,1]; <0 prior to segment, >1 past segment end
        seg_len2 = seg_dx * seg_dx + seg_dy * seg_dy
        if seg_len2 > 1e-6:
            proj = ((x - seg_x0) * seg_dx + (y - seg_y0) * seg_dy) / seg_len2
        else:
            proj = 0.0

        # -- Heading Reference --------------------------------------------------
        if mode in ("start", "finish"):
            # Direct bearing to waypoint for alignment and hold
            psi_ref = math.atan2(wp_y - y, wp_x - x)
        else:
            # Pure LOS: track the active segment line
            psi_ref = chi_p - math.atan2(e_ct, delta)

        psi_ref = wrap_angle(psi_ref)
        heading_error = wrap_angle(psi_ref - yaw)
        seg_len = math.sqrt(seg_len2)
        u_ref = self.nominal_speed(mode, distance_to_wp, pos_tol, seg_len)

        msg = Float64MultiArray()
        msg.data = [
            float(psi_ref),
            float(u_ref),
            float(e_ct),
            float(chi_p),
            float(distance_to_wp),
            float(delta),
            float(heading_error),
            float(wp_id),
            float(mode_code),
            float(pos_tol),
            float(wp_x),
            float(wp_y),
        ]
        self.guidance_pub.publish(msg)

        self.maybe_log_debug(
            wp_id,
            mode,
            x,
            y,
            yaw,
            speed,
            wp_x,
            wp_y,
            chi_p,
            e_ct,
            delta,
            psi_ref,
            heading_error,
            u_ref,
            distance_to_wp,
            proj=proj,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LOSGuidance()
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