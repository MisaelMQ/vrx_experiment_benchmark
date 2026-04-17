#!/usr/bin/env python3

import math
import random
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from vrx_experiment_benchmark.route_utils import wrap_angle, quaternion_to_yaw


class ControllerMPPI(Node):
    def __init__(self) -> None:
        super().__init__("controller_mppi")

        # Interfaces
        self.declare_parameter("state_topic", "/wamv/navigation/state2d")
        self.declare_parameter("guidance_topic", "/wamv/control/guidance_cmd")
        self.declare_parameter("active_waypoint_meta_topic", "/wamv/navigation/active_waypoint_meta")
        self.declare_parameter("thruster_cmd_topic", "/wamv/control/thruster_cmd")
        self.declare_parameter("controller_debug_topic", "/wamv/control/controller_debug")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("fixed_thruster_angle", 0.0)
        self.declare_parameter("debug_enabled", True)
        self.declare_parameter("debug_interval_sec", 1.0)

        # MPPI core
        self.declare_parameter("horizon_steps", 18)
        self.declare_parameter("dt", 0.15)
        self.declare_parameter("num_samples", 192)
        self.declare_parameter("lambda_temp", 18.0)
        self.declare_parameter("sample_smoothing", 0.72)
        self.declare_parameter("noise_sigma_left", 120.0)
        self.declare_parameter("noise_sigma_right", 120.0)
        self.declare_parameter("noise_sigma_left_hold", 70.0)
        self.declare_parameter("noise_sigma_right_hold", 70.0)
        self.declare_parameter("max_delta_thrust_per_step", 160.0)
        self.declare_parameter("warm_start_with_last_cmd", True)
        self.declare_parameter("random_seed", 7)

        # Vehicle model
        self.declare_parameter("mass_eff", 260.0)
        self.declare_parameter("iz_eff", 320.0)
        self.declare_parameter("thruster_half_spacing_m", 0.98)
        self.declare_parameter("surge_drag_linear", 100.0)
        self.declare_parameter("surge_drag_quadratic", 150.0)
        self.declare_parameter("yaw_drag_linear", 800.0)
        self.declare_parameter("yaw_drag_quadratic", 800.0)
        self.declare_parameter("surge_accel_limit", 2.5)
        self.declare_parameter("yaw_accel_limit", 2.8)
        self.declare_parameter("prediction_speed_limit", 6.5)
        self.declare_parameter("prediction_yaw_rate_limit", 1.2)

        # Constraints
        self.declare_parameter("min_thrust", -1000.0)
        self.declare_parameter("max_thrust", 1000.0)
        self.declare_parameter("reverse_thrust_scale", 0.85)
        self.declare_parameter("finish_brake_distance", 4.0)
        self.declare_parameter("completion_zero_u_ref", 0.05)

        # Cost weights: transit
        self.declare_parameter("w_cte", 24.0)
        self.declare_parameter("w_heading", 16.0)
        self.declare_parameter("w_speed", 8.0)
        self.declare_parameter("w_yaw_rate", 2.5)
        self.declare_parameter("w_progress", 3.0)
        self.declare_parameter("w_wp_distance", 1.0)
        self.declare_parameter("w_control", 0.010)
        self.declare_parameter("w_control_delta", 0.22)
        self.declare_parameter("w_saturation", 30.0)
        self.declare_parameter("w_reverse", 0.6)
        self.declare_parameter("terminal_multiplier", 3.0)

        # Cost weights: hold / docking-like behavior at start-finish
        self.declare_parameter("w_cte_hold", 10.0)
        self.declare_parameter("w_heading_hold", 24.0)
        self.declare_parameter("w_speed_hold", 10.0)
        self.declare_parameter("w_yaw_rate_hold", 4.0)
        self.declare_parameter("w_progress_hold", 1.0)
        self.declare_parameter("w_wp_distance_hold", 10.0)
        self.declare_parameter("w_control_hold", 0.014)
        self.declare_parameter("w_control_delta_hold", 0.34)
        self.declare_parameter("w_saturation_hold", 36.0)
        self.declare_parameter("w_reverse_hold", 0.3)
        self.declare_parameter("terminal_multiplier_hold", 4.0)

        self.state_topic = str(self.get_parameter("state_topic").value)
        self.guidance_topic = str(self.get_parameter("guidance_topic").value)
        self.active_waypoint_meta_topic = str(self.get_parameter("active_waypoint_meta_topic").value)
        self.thruster_cmd_topic = str(self.get_parameter("thruster_cmd_topic").value)
        self.controller_debug_topic = str(self.get_parameter("controller_debug_topic").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.fixed_thruster_angle = float(self.get_parameter("fixed_thruster_angle").value)
        self.debug_enabled = bool(self.get_parameter("debug_enabled").value)
        self.debug_interval_sec = float(self.get_parameter("debug_interval_sec").value)

        self.horizon_steps = int(self.get_parameter("horizon_steps").value)
        self.dt = float(self.get_parameter("dt").value)
        self.num_samples = int(self.get_parameter("num_samples").value)
        self.lambda_temp = float(self.get_parameter("lambda_temp").value)
        self.sample_smoothing = float(self.get_parameter("sample_smoothing").value)
        self.noise_sigma_left = float(self.get_parameter("noise_sigma_left").value)
        self.noise_sigma_right = float(self.get_parameter("noise_sigma_right").value)
        self.noise_sigma_left_hold = float(self.get_parameter("noise_sigma_left_hold").value)
        self.noise_sigma_right_hold = float(self.get_parameter("noise_sigma_right_hold").value)
        self.max_delta_thrust_per_step = float(self.get_parameter("max_delta_thrust_per_step").value)
        self.warm_start_with_last_cmd = bool(self.get_parameter("warm_start_with_last_cmd").value)
        self.random_seed = int(self.get_parameter("random_seed").value)

        self.mass_eff = float(self.get_parameter("mass_eff").value)
        self.iz_eff = float(self.get_parameter("iz_eff").value)
        self.thruster_half_spacing_m = float(self.get_parameter("thruster_half_spacing_m").value)
        self.surge_drag_linear = float(self.get_parameter("surge_drag_linear").value)
        self.surge_drag_quadratic = float(self.get_parameter("surge_drag_quadratic").value)
        self.yaw_drag_linear = float(self.get_parameter("yaw_drag_linear").value)
        self.yaw_drag_quadratic = float(self.get_parameter("yaw_drag_quadratic").value)
        self.surge_accel_limit = float(self.get_parameter("surge_accel_limit").value)
        self.yaw_accel_limit = float(self.get_parameter("yaw_accel_limit").value)
        self.prediction_speed_limit = float(self.get_parameter("prediction_speed_limit").value)
        self.prediction_yaw_rate_limit = float(self.get_parameter("prediction_yaw_rate_limit").value)

        self.min_thrust = float(self.get_parameter("min_thrust").value)
        self.max_thrust = float(self.get_parameter("max_thrust").value)
        self.reverse_thrust_scale = float(self.get_parameter("reverse_thrust_scale").value)
        self.finish_brake_distance = float(self.get_parameter("finish_brake_distance").value)
        self.completion_zero_u_ref = float(self.get_parameter("completion_zero_u_ref").value)

        self.w_cte = float(self.get_parameter("w_cte").value)
        self.w_heading = float(self.get_parameter("w_heading").value)
        self.w_speed = float(self.get_parameter("w_speed").value)
        self.w_yaw_rate = float(self.get_parameter("w_yaw_rate").value)
        self.w_progress = float(self.get_parameter("w_progress").value)
        self.w_wp_distance = float(self.get_parameter("w_wp_distance").value)
        self.w_control = float(self.get_parameter("w_control").value)
        self.w_control_delta = float(self.get_parameter("w_control_delta").value)
        self.w_saturation = float(self.get_parameter("w_saturation").value)
        self.w_reverse = float(self.get_parameter("w_reverse").value)
        self.terminal_multiplier = float(self.get_parameter("terminal_multiplier").value)

        self.w_cte_hold = float(self.get_parameter("w_cte_hold").value)
        self.w_heading_hold = float(self.get_parameter("w_heading_hold").value)
        self.w_speed_hold = float(self.get_parameter("w_speed_hold").value)
        self.w_yaw_rate_hold = float(self.get_parameter("w_yaw_rate_hold").value)
        self.w_progress_hold = float(self.get_parameter("w_progress_hold").value)
        self.w_wp_distance_hold = float(self.get_parameter("w_wp_distance_hold").value)
        self.w_control_hold = float(self.get_parameter("w_control_hold").value)
        self.w_control_delta_hold = float(self.get_parameter("w_control_delta_hold").value)
        self.w_saturation_hold = float(self.get_parameter("w_saturation_hold").value)
        self.w_reverse_hold = float(self.get_parameter("w_reverse_hold").value)
        self.terminal_multiplier_hold = float(self.get_parameter("terminal_multiplier_hold").value)

        self.state_msg: Optional[Odometry] = None
        self.guidance_msg: Optional[Float64MultiArray] = None
        self.meta_msg: Optional[Float64MultiArray] = None
        self.last_report_time_sec: float = 0.0
        self.last_left_cmd: float = 0.0
        self.last_right_cmd: float = 0.0
        self.last_best_cost: float = float("nan")

        self.nominal_left: List[float] = [0.0 for _ in range(self.horizon_steps)]
        self.nominal_right: List[float] = [0.0 for _ in range(self.horizon_steps)]

        self.rng = random.Random(self.random_seed)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Odometry, self.state_topic, self.state_callback, qos)
        self.create_subscription(Float64MultiArray, self.guidance_topic, self.guidance_callback, qos)
        self.create_subscription(Float64MultiArray, self.active_waypoint_meta_topic, self.meta_callback, qos)

        self.cmd_pub = self.create_publisher(Float64MultiArray, self.thruster_cmd_topic, qos)
        self.debug_pub = self.create_publisher(Float64MultiArray, self.controller_debug_topic, qos)

        self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self.timer_callback)

        self.get_logger().info(
            "controller_mppi started | samples=%d horizon=%d dt=%.2f "
            "max_thrust=%.0f lambda=%.1f sigma_L/R=%.0f/%.0f rate=%.0fHz" % (
                self.num_samples,
                self.horizon_steps,
                self.dt,
                self.max_thrust,
                self.lambda_temp,
                self.noise_sigma_left,
                self.noise_sigma_right,
                self.publish_rate_hz,
            )
        )

    def state_callback(self, msg: Odometry) -> None:
        self.state_msg = msg

    def guidance_callback(self, msg: Float64MultiArray) -> None:
        self.guidance_msg = msg

    def meta_callback(self, msg: Float64MultiArray) -> None:
        self.meta_msg = msg

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def clamp_thrust(self, value: float) -> float:
        lo = self.min_thrust * self.reverse_thrust_scale if value < 0.0 else self.min_thrust
        return max(lo, min(self.max_thrust, value))

    def clamp_abs(self, value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def rate_limit(self, target: float, previous: float) -> float:
        delta = target - previous
        delta = self.clamp_abs(delta, self.max_delta_thrust_per_step)
        return self.clamp_thrust(previous + delta)

    def current_state_tuple(self) -> Optional[Tuple[float, float, float, float, float]]:
        if self.state_msg is None:
            return None
        x = float(self.state_msg.pose.pose.position.x)
        y = float(self.state_msg.pose.pose.position.y)
        q = self.state_msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        vx = float(self.state_msg.twist.twist.linear.x)
        vy = float(self.state_msg.twist.twist.linear.y)
        # state2d stores world-frame planar velocity; project onto surge axis
        u = math.cos(yaw) * vx + math.sin(yaw) * vy
        r = float(self.state_msg.twist.twist.angular.z)
        return (x, y, yaw, u, r)

    def get_meta_fields(self) -> Optional[Tuple[float, float, float, float, int, int, float, float, float]]:
        if self.meta_msg is None or len(self.meta_msg.data) < 15:
            return None
        d = self.meta_msg.data
        seg_x0 = float(d[6])
        seg_y0 = float(d[7])
        seg_x1 = float(d[8])
        seg_y1 = float(d[9])
        wp_id = int(d[0])
        mode_code = int(d[1])
        pos_tol = float(d[2])
        hold_elapsed = float(d[12])
        completed = int(d[14])
        return (seg_x0, seg_y0, seg_x1, seg_y1, wp_id, mode_code, pos_tol, hold_elapsed, float(completed))

    def is_hold_mode(self, mode_code: int) -> bool:
        return mode_code in (0, 2)

    def weights_for_mode(self, mode_code: int) -> dict:
        if self.is_hold_mode(mode_code):
            return {
                "cte": self.w_cte_hold,
                "heading": self.w_heading_hold,
                "speed": self.w_speed_hold,
                "yaw_rate": self.w_yaw_rate_hold,
                "progress": self.w_progress_hold,
                "wp_dist": self.w_wp_distance_hold,
                "control": self.w_control_hold,
                "control_delta": self.w_control_delta_hold,
                "saturation": self.w_saturation_hold,
                "reverse": self.w_reverse_hold,
                "terminal": self.terminal_multiplier_hold,
            }
        return {
            "cte": self.w_cte,
            "heading": self.w_heading,
            "speed": self.w_speed,
            "yaw_rate": self.w_yaw_rate,
            "progress": self.w_progress,
            "wp_dist": self.w_wp_distance,
            "control": self.w_control,
            "control_delta": self.w_control_delta,
            "saturation": self.w_saturation,
            "reverse": self.w_reverse,
            "terminal": self.terminal_multiplier,
        }

    def model_step(self, x: float, y: float, psi: float, u: float, r: float, tl: float, tr: float) -> Tuple[float, float, float, float, float]:
        common = tl + tr
        differential = tr - tl

        du = (
            (common / max(self.mass_eff, 1e-6))
            - (self.surge_drag_linear / max(self.mass_eff, 1e-6)) * u
            - (self.surge_drag_quadratic / max(self.mass_eff, 1e-6)) * abs(u) * u
        )
        dr = (
            (self.thruster_half_spacing_m * differential / max(self.iz_eff, 1e-6))
            - (self.yaw_drag_linear / max(self.iz_eff, 1e-6)) * r
            - (self.yaw_drag_quadratic / max(self.iz_eff, 1e-6)) * abs(r) * r
        )

        du = self.clamp_abs(du, self.surge_accel_limit)
        dr = self.clamp_abs(dr, self.yaw_accel_limit)

        u_next = self.clamp_abs(u + self.dt * du, self.prediction_speed_limit)
        r_next = self.clamp_abs(r + self.dt * dr, self.prediction_yaw_rate_limit)
        psi_next = wrap_angle(psi + self.dt * r_next)
        x_next = x + self.dt * u_next * math.cos(psi_next)
        y_next = y + self.dt * u_next * math.sin(psi_next)
        return (x_next, y_next, psi_next, u_next, r_next)

    def stage_cost(
        self,
        x: float,
        y: float,
        psi: float,
        u: float,
        r: float,
        tl: float,
        tr: float,
        prev_tl: float,
        prev_tr: float,
        psi_ref: float,
        u_ref: float,
        chi_p: float,
        wp_x: float,
        wp_y: float,
        seg_x0: float,
        seg_y0: float,
        mode_code: int,
        pos_tol: float,
        initial_wp_dist: float,
    ) -> float:
        w = self.weights_for_mode(mode_code)

        e_psi = wrap_angle(psi_ref - psi)
        e_ct = -math.sin(chi_p) * (x - seg_x0) + math.cos(chi_p) * (y - seg_y0)
        dist_wp = math.hypot(wp_x - x, wp_y - y)
        progress = max(0.0, initial_wp_dist - dist_wp)

        thrust_scale = max(self.max_thrust, 1.0)
        control_norm = (tl * tl + tr * tr) / (thrust_scale * thrust_scale)
        dleft = (tl - prev_tl) / thrust_scale
        dright = (tr - prev_tr) / thrust_scale
        dcontrol_norm = dleft * dleft + dright * dright

        sat_margin_l = max(0.0, abs(tl) / thrust_scale - 0.92)
        sat_margin_r = max(0.0, abs(tr) / thrust_scale - 0.92)
        sat_pen = sat_margin_l * sat_margin_l + sat_margin_r * sat_margin_r

        reverse_pen = 0.0
        if tl < 0.0:
            reverse_pen += abs(tl) / thrust_scale
        if tr < 0.0:
            reverse_pen += abs(tr) / thrust_scale

        dist_norm = dist_wp / max(pos_tol + 2.0, 1.0)
        if mode_code == 2 and dist_wp < self.finish_brake_distance:
            dist_norm *= 1.4

        return (
            w["cte"] * e_ct * e_ct
            + w["heading"] * e_psi * e_psi
            + w["speed"] * (u - u_ref) * (u - u_ref)
            + w["yaw_rate"] * r * r
            + w["wp_dist"] * dist_norm * dist_norm
            - w["progress"] * progress
            + w["control"] * control_norm
            + w["control_delta"] * dcontrol_norm
            + w["saturation"] * sat_pen
            + w["reverse"] * reverse_pen
        )

    def terminal_cost(
        self,
        x: float,
        y: float,
        psi: float,
        u: float,
        psi_ref: float,
        chi_p: float,
        wp_x: float,
        wp_y: float,
        seg_x0: float,
        seg_y0: float,
        mode_code: int,
        pos_tol: float,
    ) -> float:
        w = self.weights_for_mode(mode_code)
        e_psi = wrap_angle(psi_ref - psi)
        e_ct = -math.sin(chi_p) * (x - seg_x0) + math.cos(chi_p) * (y - seg_y0)
        dist_wp = math.hypot(wp_x - x, wp_y - y)
        dist_norm = dist_wp / max(pos_tol + 1.5, 1.0)
        return w["terminal"] * (w["cte"] * e_ct * e_ct + w["heading"] * e_psi * e_psi + 0.5 * w["wp_dist"] * dist_norm * dist_norm + 0.5 * w["speed"] * u * u)

    def maybe_reset_nominal(self, u_ref: float, mode_code: int) -> None:
        if not self.warm_start_with_last_cmd:
            return
        if self.horizon_steps <= 0:
            return

        base = self.mass_eff * max(u_ref, 0.0) * 0.45
        if self.is_hold_mode(mode_code):
            base *= 0.70
        base = min(base, 0.55 * self.max_thrust)

        if abs(self.last_left_cmd) < 1.0 and abs(self.last_right_cmd) < 1.0 and base > 1.0:
            for i in range(self.horizon_steps):
                self.nominal_left[i] = base
                self.nominal_right[i] = base

    def compute_mppi_command(self) -> Optional[Tuple[float, float, float, float, float, float, float, float, int]]:
        if self.guidance_msg is None or self.state_msg is None or self.meta_msg is None:
            return None
        g = self.guidance_msg.data
        if len(g) < 12:
            return None
        meta = self.get_meta_fields()
        if meta is None:
            return None

        state = self.current_state_tuple()
        if state is None:
            return None

        x0, y0, psi0, u0, r0 = state
        seg_x0, seg_y0, _seg_x1, _seg_y1, _wp_id, mode_code_meta, pos_tol_meta, _hold_elapsed, completed = meta

        psi_ref = float(g[0])
        u_ref = float(g[1])
        e_ct_meas = float(g[2])
        chi_p = float(g[3])
        dist_to_wp = float(g[4])
        heading_error_from_guidance = float(g[6])
        mode_code = int(g[8]) if len(g) > 8 else mode_code_meta
        pos_tol = float(g[9]) if len(g) > 9 else pos_tol_meta
        wp_x = float(g[10])
        wp_y = float(g[11])

        if int(completed) == 1:
            return (0.0, 0.0, psi_ref, psi0, 0.0, 0.0, e_ct_meas, heading_error_from_guidance, mode_code)

        if abs(u_ref) <= self.completion_zero_u_ref and self.is_hold_mode(mode_code) and dist_to_wp <= pos_tol:
            return (0.0, 0.0, psi_ref, psi0, 0.0, 0.0, e_ct_meas, heading_error_from_guidance, mode_code)

        self.maybe_reset_nominal(u_ref, mode_code)

        sigma_left = self.noise_sigma_left_hold if self.is_hold_mode(mode_code) else self.noise_sigma_left
        sigma_right = self.noise_sigma_right_hold if self.is_hold_mode(mode_code) else self.noise_sigma_right

        eps_left_acc = [0.0 for _ in range(self.horizon_steps)]
        eps_right_acc = [0.0 for _ in range(self.horizon_steps)]
        weight_sum = 0.0
        sample_records = []
        best_cost = float("inf")
        best_first_left = 0.0
        best_first_right = 0.0

        initial_wp_dist = max(dist_to_wp, math.hypot(wp_x - x0, wp_y - y0))

        for _ in range(self.num_samples):
            x = x0
            y = y0
            psi = psi0
            u = u0
            r = r0
            prev_l = self.last_left_cmd
            prev_r = self.last_right_cmd
            total_cost = 0.0
            eps_l_seq = [0.0 for _ in range(self.horizon_steps)]
            eps_r_seq = [0.0 for _ in range(self.horizon_steps)]
            noise_l = 0.0
            noise_r = 0.0
            first_l = 0.0
            first_r = 0.0

            for t in range(self.horizon_steps):
                noise_l = self.sample_smoothing * noise_l + math.sqrt(max(1.0 - self.sample_smoothing * self.sample_smoothing, 1e-6)) * self.rng.gauss(0.0, sigma_left)
                noise_r = self.sample_smoothing * noise_r + math.sqrt(max(1.0 - self.sample_smoothing * self.sample_smoothing, 1e-6)) * self.rng.gauss(0.0, sigma_right)

                tl = self.clamp_thrust(self.nominal_left[t] + noise_l)
                tr = self.clamp_thrust(self.nominal_right[t] + noise_r)

                if t == 0:
                    tl = self.rate_limit(tl, self.last_left_cmd)
                    tr = self.rate_limit(tr, self.last_right_cmd)
                    first_l = tl
                    first_r = tr

                eps_l_seq[t] = tl - self.nominal_left[t]
                eps_r_seq[t] = tr - self.nominal_right[t]

                x, y, psi, u, r = self.model_step(x, y, psi, u, r, tl, tr)
                total_cost += self.stage_cost(
                    x, y, psi, u, r,
                    tl, tr, prev_l, prev_r,
                    psi_ref, u_ref, chi_p,
                    wp_x, wp_y, seg_x0, seg_y0,
                    mode_code, pos_tol, initial_wp_dist,
                )

                prev_l = tl
                prev_r = tr

            total_cost += self.terminal_cost(
                x, y, psi, u,
                psi_ref, chi_p,
                wp_x, wp_y, seg_x0, seg_y0,
                mode_code, pos_tol,
            )

            sample_records.append((total_cost, eps_l_seq, eps_r_seq, first_l, first_r))
            if total_cost < best_cost:
                best_cost = total_cost
                best_first_left = first_l
                best_first_right = first_r

        if not sample_records:
            return None

        min_cost = min(item[0] for item in sample_records)
        lam = max(self.lambda_temp, 1e-6)

        for total_cost, eps_l_seq, eps_r_seq, _first_l, _first_r in sample_records:
            weight = math.exp(-(total_cost - min_cost) / lam)
            weight_sum += weight
            for t in range(self.horizon_steps):
                eps_left_acc[t] += weight * eps_l_seq[t]
                eps_right_acc[t] += weight * eps_r_seq[t]

        if weight_sum <= 1e-9:
            cmd_l = best_first_left
            cmd_r = best_first_right
        else:
            for t in range(self.horizon_steps):
                self.nominal_left[t] = self.clamp_thrust(self.nominal_left[t] + eps_left_acc[t] / weight_sum)
                self.nominal_right[t] = self.clamp_thrust(self.nominal_right[t] + eps_right_acc[t] / weight_sum)

            cmd_l = self.rate_limit(self.nominal_left[0], self.last_left_cmd)
            cmd_r = self.rate_limit(self.nominal_right[0], self.last_right_cmd)
            self.nominal_left[0] = cmd_l
            self.nominal_right[0] = cmd_r

        self.last_best_cost = best_cost

        # Shift warm-start horizon
        if self.horizon_steps > 1:
            for t in range(self.horizon_steps - 1):
                self.nominal_left[t] = self.nominal_left[t + 1]
                self.nominal_right[t] = self.nominal_right[t + 1]
            self.nominal_left[-1] = self.nominal_left[-2]
            self.nominal_right[-1] = self.nominal_right[-2]

        self.last_left_cmd = cmd_l
        self.last_right_cmd = cmd_r

        common = 0.5 * (cmd_l + cmd_r)
        differential = 0.5 * (cmd_r - cmd_l)
        return (cmd_l, cmd_r, psi_ref, psi0, common, differential, e_ct_meas, heading_error_from_guidance, mode_code)

    def maybe_log_debug(
        self,
        psi_ref: float,
        yaw: float,
        yaw_error: float,
        yaw_rate: float,
        u_ref: float,
        common: float,
        differential: float,
        left_thrust: float,
        right_thrust: float,
        e_ct: float,
        mode_code: int,
    ) -> None:
        if not self.debug_enabled:
            return
        now = self.now_sec()
        if (now - self.last_report_time_sec) < max(self.debug_interval_sec, 0.05):
            return
        self.last_report_time_sec = now
        self.get_logger().info(
            "MPPI[m:%d]|psi:%.2f/%.2f(e:%.2f,r:%.2f)|u:%.1f|c:%.1f,d:%.1f|th:%.1f,%.1f|e_ct:%.1f|J:%.1f" % (
                mode_code,
                psi_ref,
                yaw,
                yaw_error,
                yaw_rate,
                u_ref,
                common,
                differential,
                left_thrust,
                right_thrust,
                e_ct,
                self.last_best_cost,
            )
        )

    def timer_callback(self) -> None:
        result = self.compute_mppi_command()
        if result is None or self.state_msg is None or self.guidance_msg is None:
            return

        left_thrust, right_thrust, psi_ref, yaw, common, differential, e_ct, heading_error_from_guidance, mode_code = result
        yaw_rate = float(self.state_msg.twist.twist.angular.z)
        yaw_error = wrap_angle(psi_ref - yaw)
        u_ref = float(self.guidance_msg.data[1])

        msg = Float64MultiArray()
        msg.data = [
            float(left_thrust),
            float(right_thrust),
            float(self.fixed_thruster_angle),
            float(self.fixed_thruster_angle),
        ]
        self.cmd_pub.publish(msg)

        dbg = Float64MultiArray()
        dbg.data = [
            float(psi_ref),
            float(yaw),
            float(yaw_error),
            float(yaw_rate),
            float(u_ref),
            float(common),
            float(differential),
            float(left_thrust),
            float(right_thrust),
            float(e_ct),
            float(heading_error_from_guidance),
            float(mode_code),
        ]
        self.debug_pub.publish(dbg)

        self.maybe_log_debug(
            psi_ref,
            yaw,
            yaw_error,
            yaw_rate,
            u_ref,
            common,
            differential,
            left_thrust,
            right_thrust,
            e_ct,
            mode_code,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerMPPI()
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