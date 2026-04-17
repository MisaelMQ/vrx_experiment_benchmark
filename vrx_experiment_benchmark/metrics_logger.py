#!/usr/bin/env python3
"""metrics_logger — navigation metrics recording node.

Passive subscriptions (does not interact or publish control logic):
  * /wamv/navigation/state2d              → Odometry
  * /wamv/navigation/route_status         → Float64MultiArray [idx, total, dist, ready, completed]
  * /wamv/navigation/active_waypoint_meta → Float64MultiArray (15 fields)
  * /wamv/control/guidance_cmd            → Float64MultiArray (12 fields)
  * /wamv/control/controller_debug        → Float64MultiArray (12 fields)
  * /wamv/control/thruster_cmd            → Float64MultiArray [L, R, L_pos, R_pos]

Outputs:
  metrics/raw/<run_tag>_timeseries.csv    — 10Hz tabular time sequence
  metrics/raw/<run_tag>_events.csv        — sparse navigation state events
  metrics/summary/<run_tag>_summary.csv   — single-row route summary metrics at termination
"""

import csv
import math
import os
from datetime import datetime
from typing import Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from vrx_experiment_benchmark.route_utils import quaternion_to_yaw, CODE_TO_MODE

# Topic message index constants
# route_status [idx, total, dist, ready, completed]
_RS_IDX = 0; _RS_TOTAL = 1; _RS_DIST = 2; _RS_READY = 3; _RS_DONE = 4

# active_waypoint_meta (15 campos)
_M_WP_ID = 0; _M_MODE = 1; _M_TOL = 2
_M_WP_X = 3; _M_WP_Y = 4
_M_SEG_X0 = 6; _M_SEG_Y0 = 7; _M_SEG_X1 = 8; _M_SEG_Y1 = 9
_M_HOLD_ELAPSED = 12; _M_HOLD_REQUIRED = 13; _M_COMPLETED = 14

# guidance_cmd [psi_ref, u_ref, e_ct, chi_p, dist_wp, delta, heading_error,
#               wp_id, mode_code, pos_tol, wp_x, wp_y]
_G_PSI = 0; _G_UREF = 1; _G_ECT = 2; _G_CHIP = 3
_G_DIST = 4; _G_DELTA = 5; _G_HERR = 6

# controller_debug [psi_ref, yaw, yaw_err, yaw_rate, u_ref, common,
#                   diff, left, right, e_ct, heading_err, mode_code]
_CD_LEFT = 7; _CD_RIGHT = 8

# thruster_cmd [L, R, L_pos, R_pos]
_T_L = 0; _T_R = 1; _T_LP = 2; _T_RP = 3

_TIMESERIES_HEADER = [
    "t",
    "active_idx", "total_waypoints", "route_completed",
    "x", "y", "yaw", "vx", "vy", "speed",
    "wp_id", "mode_code", "pos_tol", "wp_x", "wp_y",
    "dist_to_wp", "e_ct", "chi_p", "delta", "psi_ref", "heading_error", "u_ref",
    "left_thrust", "right_thrust", "left_angle", "right_angle",
]

_EVENTS_HEADER = [
    "t", "event", "active_idx", "wp_id", "mode", "dist", "proj", "hold_elapsed",
]

_SUMMARY_HEADER = [
    "run_tag", "route_name", "completed", "total_time_sec",
    "distance_travelled_m", "straight_line_dist_m", "path_efficiency",
    "mean_speed_mps", "max_speed_mps",
    "mean_abs_cte_m", "rms_cte_m", "max_abs_cte_m",
    "waypoints_completed",
    "mean_u_ref", "mean_left_thrust", "mean_right_thrust",
]


class MetricsLogger(Node):
    def __init__(self) -> None:
        super().__init__("metrics_logger")

        # Node parameters
        self.declare_parameter("state_topic", "/wamv/navigation/state2d")
        self.declare_parameter("route_status_topic", "/wamv/navigation/route_status")
        self.declare_parameter("active_waypoint_meta_topic", "/wamv/navigation/active_waypoint_meta")
        self.declare_parameter("guidance_topic", "/wamv/control/guidance_cmd")
        self.declare_parameter("controller_debug_topic", "/wamv/control/controller_debug")
        self.declare_parameter("thruster_cmd_topic", "/wamv/control/thruster_cmd")
        self.declare_parameter("flush_rate_hz", 10.0)
        self.declare_parameter("table_print_interval_sec", 1.0)
        self.declare_parameter("run_tag", "")
        self.declare_parameter("output_root", "metrics")

        state_topic = str(self.get_parameter("state_topic").value)
        route_status_topic = str(self.get_parameter("route_status_topic").value)
        meta_topic = str(self.get_parameter("active_waypoint_meta_topic").value)
        guidance_topic = str(self.get_parameter("guidance_topic").value)
        ctrl_debug_topic = str(self.get_parameter("controller_debug_topic").value)
        thruster_topic = str(self.get_parameter("thruster_cmd_topic").value)

        self.flush_rate_hz = float(self.get_parameter("flush_rate_hz").value)
        self.table_interval = float(self.get_parameter("table_print_interval_sec").value)

        run_tag_param = str(self.get_parameter("run_tag").value).strip()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_tag = (
            f"{run_tag_param}_{timestamp}" if run_tag_param else timestamp
        )

        output_root = str(self.get_parameter("output_root").value)

        # Resolve output directory relative to package share
        self.raw_dir, self.summary_dir = self._resolve_output_dirs(output_root)

        # Message buffers
        self.state_msg: Optional[Odometry] = None
        self.route_status_msg: Optional[Float64MultiArray] = None
        self.meta_msg: Optional[Float64MultiArray] = None
        self.guidance_msg: Optional[Float64MultiArray] = None
        self.ctrl_debug_msg: Optional[Float64MultiArray] = None
        self.thruster_msg: Optional[Float64MultiArray] = None

        # Internal state for event detection
        self._prev_active_idx: Optional[int] = None
        self._prev_inside: Optional[bool] = None
        self._hold_started_logged: bool = False
        self._route_done_logged: bool = False
        self._start_time_sec: Optional[float] = None
        self._last_table_sec: float = 0.0

        # Accumulators for route summary
        self._n_rows: int = 0
        self._sum_speed: float = 0.0
        self._max_speed: float = 0.0
        self._sum_abs_cte: float = 0.0
        self._max_abs_cte: float = 0.0
        self._sum_u_ref: float = 0.0
        self._sum_left: float = 0.0
        self._sum_right: float = 0.0
        self._dist_travelled: float = 0.0
        self._prev_xy: Optional[tuple] = None
        self._first_pos: Optional[tuple] = None   # First non-zero recorded position
        self._last_pos: Optional[tuple] = None    # Last recorded position
        self._sum_cte_sq: float = 0.0             # RMS CTE accumulator
        self._waypoints_completed: int = 0

        # Initialize CSV loggers
        ts_path = os.path.join(self.raw_dir, f"{self.run_tag}_timeseries.csv")
        ev_path = os.path.join(self.raw_dir, f"{self.run_tag}_events.csv")
        self._summary_path = os.path.join(self.summary_dir, f"{self.run_tag}_summary.csv")

        self._ts_file = open(ts_path, "w", newline="", buffering=1)
        self._ev_file = open(ev_path, "w", newline="", buffering=1)
        self._ts_writer = csv.writer(self._ts_file)
        self._ev_writer = csv.writer(self._ev_file)
        self._ts_writer.writerow(_TIMESERIES_HEADER)
        self._ev_writer.writerow(_EVENTS_HEADER)

        self.get_logger().info(
            f"[metrics] run_tag={self.run_tag}\n"
            f"  timeseries: {ts_path}\n"
            f"  events:     {ev_path}\n"
            f"  summary:    {self._summary_path}"
        )

        # Topic streams / QoS overrides
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Odometry, state_topic, self._cb_state, qos)
        self.create_subscription(Float64MultiArray, route_status_topic, self._cb_route_status, qos)
        self.create_subscription(Float64MultiArray, meta_topic, self._cb_meta, qos)
        self.create_subscription(Float64MultiArray, guidance_topic, self._cb_guidance, qos)
        self.create_subscription(Float64MultiArray, ctrl_debug_topic, self._cb_ctrl_debug, qos)
        self.create_subscription(Float64MultiArray, thruster_topic, self._cb_thruster, qos)

        period = 1.0 / max(self.flush_rate_hz, 1.0)
        self.create_timer(period, self._timer_flush)
        # Save summary checkpoint every 2s to protect against Ctrl+C
        self.create_timer(2.0, self._timer_summary_checkpoint)

    # Topic Callbacks

    def _cb_state(self, msg: Odometry) -> None:
        self.state_msg = msg

    def _cb_route_status(self, msg: Float64MultiArray) -> None:
        self.route_status_msg = msg

    def _cb_meta(self, msg: Float64MultiArray) -> None:
        self.meta_msg = msg

    def _cb_guidance(self, msg: Float64MultiArray) -> None:
        self.guidance_msg = msg

    def _cb_ctrl_debug(self, msg: Float64MultiArray) -> None:
        self.ctrl_debug_msg = msg

    def _cb_thruster(self, msg: Float64MultiArray) -> None:
        self.thruster_msg = msg

    # Main synchronization timer

    def _timer_flush(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9

        if self._start_time_sec is None:
            self._start_time_sec = now

        t = now - self._start_time_sec

        # Extract parsed state message fields
        x = y = yaw = vx = vy = speed = float("nan")
        if self.state_msg is not None:
            x = float(self.state_msg.pose.pose.position.x)
            y = float(self.state_msg.pose.pose.position.y)
            vx = float(self.state_msg.twist.twist.linear.x)
            vy = float(self.state_msg.twist.twist.linear.y)
            speed = math.hypot(vx, vy)
            q = self.state_msg.pose.pose.orientation
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        active_idx = total_wps = 0
        route_completed = False
        if self.route_status_msg is not None and len(self.route_status_msg.data) >= 5:
            d = self.route_status_msg.data
            active_idx = int(d[_RS_IDX])
            total_wps = int(d[_RS_TOTAL])
            route_completed = bool(int(d[_RS_DONE]))

        wp_id = mode_code = 0
        pos_tol = wp_x = wp_y = hold_elapsed = float("nan")
        if self.meta_msg is not None and len(self.meta_msg.data) >= 15:
            m = self.meta_msg.data
            wp_id = int(m[_M_WP_ID])
            mode_code = int(m[_M_MODE])
            pos_tol = float(m[_M_TOL])
            wp_x = float(m[_M_WP_X])
            wp_y = float(m[_M_WP_Y])
            hold_elapsed = float(m[_M_HOLD_ELAPSED])

        psi_ref = u_ref = e_ct = chi_p = dist_to_wp = delta = heading_error = float("nan")
        if self.guidance_msg is not None and len(self.guidance_msg.data) >= 7:
            g = self.guidance_msg.data
            psi_ref = float(g[_G_PSI])
            u_ref = float(g[_G_UREF])
            e_ct = float(g[_G_ECT])
            chi_p = float(g[_G_CHIP])
            dist_to_wp = float(g[_G_DIST])
            delta = float(g[_G_DELTA])
            heading_error = float(g[_G_HERR])

        left_thrust = right_thrust = float("nan")
        if self.ctrl_debug_msg is not None and len(self.ctrl_debug_msg.data) >= 9:
            left_thrust = float(self.ctrl_debug_msg.data[_CD_LEFT])
            right_thrust = float(self.ctrl_debug_msg.data[_CD_RIGHT])

        left_angle = right_angle = 0.0
        if self.thruster_msg is not None and len(self.thruster_msg.data) >= 4:
            left_angle = float(self.thruster_msg.data[_T_LP])
            right_angle = float(self.thruster_msg.data[_T_RP])

        # Accumulate tracking metrics
        if not math.isnan(speed):
            self._sum_speed += speed
            if speed > self._max_speed:
                self._max_speed = speed
        if not math.isnan(e_ct):
            abs_cte = abs(e_ct)
            self._sum_abs_cte += abs_cte
            self._sum_cte_sq += e_ct * e_ct
            if abs_cte > self._max_abs_cte:
                self._max_abs_cte = abs_cte
        if not math.isnan(u_ref):
            self._sum_u_ref += u_ref
        if not math.isnan(left_thrust):
            self._sum_left += abs(left_thrust)
        if not math.isnan(right_thrust):
            self._sum_right += abs(right_thrust)
        if not math.isnan(x) and not math.isnan(y):
            if self._prev_xy is not None:
                dx = x - self._prev_xy[0]
                dy = y - self._prev_xy[1]
                self._dist_travelled += math.hypot(dx, dy)
            self._prev_xy = (x, y)
            self._last_pos = (x, y)
            if self._first_pos is None and (abs(x) > 0.01 or abs(y) > 0.01):
                self._first_pos = (x, y)
        self._n_rows += 1
        # Periodic console log confirmation
        if self._n_rows % 50 == 0:
            self.get_logger().info(
                f"[metrics] rows_written={self._n_rows}  →  {self._ts_file.name}"
            )

        # Write timeseries output row
        self._ts_writer.writerow([
            f"{t:.3f}",
            active_idx, total_wps, int(route_completed),
            f"{x:.4f}", f"{y:.4f}", f"{yaw:.4f}",
            f"{vx:.4f}", f"{vy:.4f}", f"{speed:.4f}",
            wp_id, mode_code, f"{pos_tol:.2f}", f"{wp_x:.4f}", f"{wp_y:.4f}",
            f"{dist_to_wp:.4f}", f"{e_ct:.4f}", f"{chi_p:.4f}",
            f"{delta:.4f}", f"{psi_ref:.4f}", f"{heading_error:.4f}", f"{u_ref:.4f}",
            f"{left_thrust:.2f}", f"{right_thrust:.2f}",
            f"{left_angle:.4f}", f"{right_angle:.4f}",
        ])
        # Flush and sync to disk protecting against process interrupt
        self._ts_file.flush()
        os.fsync(self._ts_file.fileno())
        self._ev_file.flush()
        os.fsync(self._ev_file.fileno())

        # Detect and record discrete lifecycle events
        self._detect_events(t, active_idx, wp_id, mode_code, dist_to_wp,
                            pos_tol, hold_elapsed, route_completed)

        # Print compact console status table
        self._maybe_print_table(t, active_idx, wp_id, mode_code,
                                dist_to_wp, speed, u_ref, e_ct, psi_ref,
                                yaw, left_thrust, right_thrust)

        # Generate final summary closure upon route completion
        if route_completed and not self._route_done_logged:
            self._route_done_logged = True
            self._write_summary(t, total_wps, route_completed)

    # Event Detection Logic

    def _detect_events(
        self,
        t: float,
        active_idx: int,
        wp_id: int,
        mode_code: int,
        dist: float,
        pos_tol: float,
        hold_elapsed: float,
        route_completed: bool,
    ) -> None:
        mode = CODE_TO_MODE.get(mode_code, "transit")

        if not math.isnan(dist) and not math.isnan(pos_tol):
            inside = dist <= pos_tol
            proj = self._compute_proj()

            if self._prev_inside is not None:
                if inside and not self._prev_inside:
                    self._log_event(t, "entered_tolerance", active_idx, wp_id, mode, dist, proj, hold_elapsed)
                elif not inside and self._prev_inside:
                    self._log_event(t, "exited_tolerance", active_idx, wp_id, mode, dist, proj, hold_elapsed)

            if inside and hold_elapsed > 0.0 and not self._hold_started_logged:
                self._hold_started_logged = True
                self._log_event(t, "hold_started", active_idx, wp_id, mode, dist, proj, hold_elapsed)

            self._prev_inside = inside

        if self._prev_active_idx is not None and active_idx != self._prev_active_idx:
            self._log_event(t, "waypoint_reached", self._prev_active_idx, wp_id, mode,
                            dist, self._compute_proj(), hold_elapsed)
            self._waypoints_completed += 1
            self._hold_started_logged = False

        if route_completed and not self._route_done_logged:
            self._log_event(t, "route_completed", active_idx, wp_id, mode, dist,
                            self._compute_proj(), hold_elapsed)

        self._prev_active_idx = active_idx

    def _compute_proj(self) -> float:
        # Scalar projection over active route segment
        if self.meta_msg is None or len(self.meta_msg.data) < 10:
            return float("nan")
        if self.state_msg is None:
            return float("nan")

        m = self.meta_msg.data
        x = float(self.state_msg.pose.pose.position.x)
        y = float(self.state_msg.pose.pose.position.y)
        x0, y0 = float(m[_M_SEG_X0]), float(m[_M_SEG_Y0])
        x1, y1 = float(m[_M_SEG_X1]), float(m[_M_SEG_Y1])
        sdx, sdy = x1 - x0, y1 - y0
        seg_len2 = sdx * sdx + sdy * sdy
        if seg_len2 < 1e-6:
            return 0.0
        return ((x - x0) * sdx + (y - y0) * sdy) / seg_len2

    def _log_event(
        self,
        t: float,
        event: str,
        active_idx: int,
        wp_id: int,
        mode: str,
        dist: float,
        proj: float,
        hold_elapsed: float,
    ) -> None:
        self._ev_writer.writerow([
            f"{t:.3f}", event, active_idx, wp_id, mode,
            f"{dist:.4f}" if not math.isnan(dist) else "nan",
            f"{proj:.4f}" if not math.isnan(proj) else "nan",
            f"{hold_elapsed:.3f}" if not math.isnan(hold_elapsed) else "nan",
        ])
        self.get_logger().info(
            f"[metrics] EVENT {event}: idx={active_idx} wp={wp_id} mode={mode} "
            f"dist={dist:.2f} proj={proj:.3f} hold={hold_elapsed:.2f}"
        )

    # Compact Status Table Printer

    def _maybe_print_table(
        self,
        t: float,
        active_idx: int,
        wp_id: int,
        mode_code: int,
        dist: float,
        speed: float,
        u_ref: float,
        e_ct: float,
        psi_ref: float,
        yaw: float,
        left_thrust: float,
        right_thrust: float,
    ) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self._last_table_sec) < max(self.table_interval, 0.05):
            return
        self._last_table_sec = now

        mode_str = CODE_TO_MODE.get(mode_code, "?")[:3]
        self.get_logger().info(
            f"MET|t:{t:.1f}|id:{active_idx}(w:{wp_id}:{mode_str})|d:{dist:.1f}|s:{speed:.1f}/u:{u_ref:.1f}|e_ct:{e_ct:.1f}|p:{psi_ref:.2f}/y:{yaw:.2f}|th:{left_thrust:.0f},{right_thrust:.0f}"
        )

    # Summary Statistics Generation

    def _write_summary(self, elapsed: float, total_wps: int, completed: bool) -> None:
        n = max(self._n_rows, 1)

        # Straight line distance from first to last logged position
        if self._first_pos is not None and self._last_pos is not None:
            straight_line = math.hypot(
                self._last_pos[0] - self._first_pos[0],
                self._last_pos[1] - self._first_pos[1],
            )
        else:
            straight_line = 0.0

        actual_dist = max(self._dist_travelled, 1e-6)
        path_efficiency = min(straight_line / actual_dist, 1.0)
        rms_cte = math.sqrt(self._sum_cte_sq / n)

        with open(self._summary_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(_SUMMARY_HEADER)
            w.writerow([
                self.run_tag,
                self.run_tag,
                int(completed),
                f"{elapsed:.3f}",
                f"{self._dist_travelled:.3f}",
                f"{straight_line:.3f}",
                f"{path_efficiency:.4f}",
                f"{self._sum_speed / n:.4f}",
                f"{self._max_speed:.4f}",
                f"{self._sum_abs_cte / n:.4f}",
                f"{rms_cte:.4f}",
                f"{self._max_abs_cte:.4f}",
                self._waypoints_completed,
                f"{self._sum_u_ref / n:.4f}",
                f"{self._sum_left / n:.4f}",
                f"{self._sum_right / n:.4f}",
            ])

        self.get_logger().info(
            f"[metrics] Summary \u2192 {self._summary_path}  "
            f"time={elapsed:.1f}s dist={self._dist_travelled:.1f}m "
            f"straight={straight_line:.1f}m efficiency={path_efficiency:.3f} "
            f"mean_speed={self._sum_speed/n:.2f}m/s rms_cte={rms_cte:.3f}m "
            f"completed={int(completed)}"
        )

    def _timer_summary_checkpoint(self) -> None:
        # Save partial summary to preserve data when terminating process
        if self._route_done_logged:
            return  # skip if final summary is already generated
        if self._start_time_sec is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self._start_time_sec
        self._write_summary(elapsed, 0, completed=False)

    # Output Directory Resolver Logic

    def _resolve_output_dirs(self, output_root: str) -> tuple:
        # Priority order: cwd -> home -> pkg_share -> /tmp
        candidates = [
            os.path.abspath(output_root),                                    # 1. Process cwd
            os.path.join(os.path.expanduser("~"), output_root),             # 2. Home directory
        ]
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory("vrx_experiment_benchmark")
            candidates.append(os.path.join(pkg_share, output_root))         # 3. Installed package share
        except Exception:
            pass

        base = None
        for candidate in candidates:
            try:
                os.makedirs(candidate, exist_ok=True)
                test_file = os.path.join(candidate, ".write_test")
                with open(test_file, "w") as _:
                    pass
                os.remove(test_file)
                base = candidate
                break
            except OSError:
                continue

        if base is None:
            base = os.path.join("/tmp", "vrx_metrics")
            self.get_logger().warn(
                f"[metrics] Falling back to {base} for saving results"
            )

        raw_dir = os.path.join(base, "raw")
        summary_dir = os.path.join(base, "summary")
        os.makedirs(raw_dir, exist_ok=True)
        os.makedirs(summary_dir, exist_ok=True)
        return raw_dir, summary_dir

    # Shutdown Logic

    def shutdown(self) -> None:
        # Generate implicit summary if incomplete, closing all streams
        if not self._route_done_logged:
            now = self.get_clock().now().nanoseconds * 1e-9
            elapsed = now - (self._start_time_sec or now)
            self._write_summary(elapsed, 0, completed=False)

        self._ts_file.flush()
        self._ev_file.flush()
        self._ts_file.close()
        self._ev_file.close()
        self.get_logger().info("[metrics] CSV files closed.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
