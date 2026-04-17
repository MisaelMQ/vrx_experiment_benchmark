#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime
from typing import Optional, Tuple, Dict, Any

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from vrx_experiment_benchmark.route_utils import wrap_angle, quaternion_to_yaw


class _ResidualMLP:
    """Minimal wrapper for TorchScript or state_dict inference.

    Loads lazily only if use_learned_policy=True and torch is available.
    """

    def __init__(self, model_path: str, policy_format: str, input_dim: int, hidden_dim: int, device: str) -> None:
        self.available = False
        self.error: Optional[str] = None
        self.device = device
        self.policy_format = policy_format
        self.input_dim = input_dim
        self.hidden_dim = hidden_dim
        self.model_path = model_path
        self.norm_mean = None
        self.norm_std = None

        try:
            import torch
            import torch.nn as nn
        except Exception as exc:
            self.error = f"torch_import_failed: {exc}"
            return

        self.torch = torch
        self.nn = nn

        if not os.path.isfile(model_path):
            self.error = f"policy_file_not_found: {model_path}"
            return

        try:
            if policy_format.lower() == "torchscript":
                self.model = torch.jit.load(model_path, map_location=device)
                self.model.eval()
                self.available = True
                return

            class ResidualNet(nn.Module):
                def __init__(self, in_dim: int, hid_dim: int) -> None:
                    super().__init__()
                    self.net = nn.Sequential(
                        nn.Linear(in_dim, hid_dim),
                        nn.Tanh(),
                        nn.Linear(hid_dim, hid_dim),
                        nn.Tanh(),
                        nn.Linear(hid_dim, 2),
                    )

                def forward(self, x):
                    return self.net(x)

            payload = torch.load(model_path, map_location=device)
            model = ResidualNet(input_dim, hidden_dim)
            if isinstance(payload, dict) and "state_dict" in payload:
                model.load_state_dict(payload["state_dict"])
                self.norm_mean = payload.get("norm_mean", None)
                self.norm_std = payload.get("norm_std", None)
            else:
                model.load_state_dict(payload)
            model.eval()
            self.model = model
            self.available = True
        except Exception as exc:
            self.error = f"policy_load_failed: {exc}"

    def predict(self, obs):
        if not self.available:
            raise RuntimeError(self.error or "policy_unavailable")

        torch = self.torch
        with torch.no_grad():
            x = torch.tensor(obs, dtype=torch.float32, device=self.device).view(1, -1)
            if self.norm_mean is not None and self.norm_std is not None:
                mean = torch.tensor(self.norm_mean, dtype=torch.float32, device=self.device).view(1, -1)
                std = torch.tensor(self.norm_std, dtype=torch.float32, device=self.device).view(1, -1)
                x = (x - mean) / torch.clamp(std, min=1e-6)
            y = self.model(x).view(-1).cpu().numpy().tolist()
        return float(y[0]), float(y[1])


class ControllerRLResidual(Node):
    def __init__(self) -> None:
        super().__init__("controller_rl_residual")

        # Interfaces
        self.declare_parameter("state_topic", "/wamv/navigation/state2d")
        self.declare_parameter("guidance_topic", "/wamv/control/guidance_cmd")
        self.declare_parameter("active_waypoint_meta_topic", "/wamv/navigation/active_waypoint_meta")
        self.declare_parameter("base_thruster_cmd_topic", "/wamv/control/thruster_cmd_raw")
        self.declare_parameter("base_controller_debug_topic", "/wamv/control/controller_debug_raw")
        self.declare_parameter("thruster_cmd_topic", "/wamv/control/thruster_cmd")
        self.declare_parameter("controller_debug_topic", "/wamv/control/controller_debug")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("fixed_thruster_angle", 0.0)
        self.declare_parameter("debug_enabled", True)
        self.declare_parameter("debug_interval_sec", 1.0)

        # Runtime / policy mode
        self.declare_parameter("enable_residual", True)
        self.declare_parameter("shadow_mode", False)
        self.declare_parameter("use_learned_policy", False)
        self.declare_parameter("policy_format", "torchscript")  # torchscript | state_dict
        self.declare_parameter("policy_path", "mppi_rl_residual_policy.pt")
        self.declare_parameter("policy_device", "cpu")
        self.declare_parameter("policy_input_dim", 20)
        self.declare_parameter("policy_hidden_dim", 64)
        self.declare_parameter("alpha_residual", 0.50)          # Generic fallback
        # Mode-based alpha: higher authority in transit, conservative during holds
        self.declare_parameter("alpha_residual_transit",      0.50)  # Transit mode
        self.declare_parameter("alpha_residual_hold",         0.20)  # Start/finish mode (waypoint capture)
        self.declare_parameter("alpha_residual_zigzag_diff",  0.60)  # Differential scaling for short segments

        # Safety / bounds
        self.declare_parameter("max_residual_common_transit", 120.0)
        self.declare_parameter("max_residual_diff_transit", 150.0)
        self.declare_parameter("max_residual_common_hold", 60.0)
        self.declare_parameter("max_residual_diff_hold", 80.0)
        self.declare_parameter("max_delta_residual_per_step", 40.0)
        self.declare_parameter("max_total_thrust", 1000.0)
        self.declare_parameter("min_total_thrust", -1000.0)
        self.declare_parameter("reverse_thrust_scale", 0.85)

        # Heuristic fallback / bootstrap residual
        self.declare_parameter("heuristic_k_cte", 10.0)
        self.declare_parameter("heuristic_k_heading", 65.0)
        self.declare_parameter("heuristic_k_yaw_rate", 28.0)
        self.declare_parameter("heuristic_k_speed_error", 24.0)
        self.declare_parameter("heuristic_finish_brake_gain", 42.0)
        self.declare_parameter("heuristic_finish_dist_thresh", 4.0)
        self.declare_parameter("heuristic_zigzag_gain", 18.0)

        # Dataset recording
        self.declare_parameter("record_dataset", True)
        self.declare_parameter("dataset_output_root", "metrics/rl_dataset")
        self.declare_parameter("dataset_tag", "")
        self.declare_parameter("dataset_flush_rate_hz", 10.0)

        # Parameters -> attributes
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.guidance_topic = str(self.get_parameter("guidance_topic").value)
        self.active_waypoint_meta_topic = str(self.get_parameter("active_waypoint_meta_topic").value)
        self.base_thruster_cmd_topic = str(self.get_parameter("base_thruster_cmd_topic").value)
        self.base_controller_debug_topic = str(self.get_parameter("base_controller_debug_topic").value)
        self.thruster_cmd_topic = str(self.get_parameter("thruster_cmd_topic").value)
        self.controller_debug_topic = str(self.get_parameter("controller_debug_topic").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.fixed_thruster_angle = float(self.get_parameter("fixed_thruster_angle").value)
        self.debug_enabled = bool(self.get_parameter("debug_enabled").value)
        self.debug_interval_sec = float(self.get_parameter("debug_interval_sec").value)

        self.enable_residual = bool(self.get_parameter("enable_residual").value)
        self.shadow_mode = bool(self.get_parameter("shadow_mode").value)
        self.use_learned_policy = bool(self.get_parameter("use_learned_policy").value)
        self.policy_format = str(self.get_parameter("policy_format").value)
        self.policy_path = str(self.get_parameter("policy_path").value)
        self.policy_device = str(self.get_parameter("policy_device").value)
        self.policy_input_dim = int(self.get_parameter("policy_input_dim").value)
        self.policy_hidden_dim = int(self.get_parameter("policy_hidden_dim").value)
        self.alpha_residual              = float(self.get_parameter("alpha_residual").value)
        self.alpha_residual_transit      = float(self.get_parameter("alpha_residual_transit").value)
        self.alpha_residual_hold         = float(self.get_parameter("alpha_residual_hold").value)
        self.alpha_residual_zigzag_diff  = float(self.get_parameter("alpha_residual_zigzag_diff").value)

        self.max_residual_common_transit = float(self.get_parameter("max_residual_common_transit").value)
        self.max_residual_diff_transit = float(self.get_parameter("max_residual_diff_transit").value)
        self.max_residual_common_hold = float(self.get_parameter("max_residual_common_hold").value)
        self.max_residual_diff_hold = float(self.get_parameter("max_residual_diff_hold").value)
        self.max_delta_residual_per_step = float(self.get_parameter("max_delta_residual_per_step").value)
        self.max_total_thrust = float(self.get_parameter("max_total_thrust").value)
        self.min_total_thrust = float(self.get_parameter("min_total_thrust").value)
        self.reverse_thrust_scale = float(self.get_parameter("reverse_thrust_scale").value)

        self.heuristic_k_cte = float(self.get_parameter("heuristic_k_cte").value)
        self.heuristic_k_heading = float(self.get_parameter("heuristic_k_heading").value)
        self.heuristic_k_yaw_rate = float(self.get_parameter("heuristic_k_yaw_rate").value)
        self.heuristic_k_speed_error = float(self.get_parameter("heuristic_k_speed_error").value)
        self.heuristic_finish_brake_gain = float(self.get_parameter("heuristic_finish_brake_gain").value)
        self.heuristic_finish_dist_thresh = float(self.get_parameter("heuristic_finish_dist_thresh").value)
        self.heuristic_zigzag_gain = float(self.get_parameter("heuristic_zigzag_gain").value)

        self.record_dataset = bool(self.get_parameter("record_dataset").value)
        self.dataset_output_root = str(self.get_parameter("dataset_output_root").value)
        self.dataset_tag = str(self.get_parameter("dataset_tag").value).strip()
        self.dataset_flush_rate_hz = float(self.get_parameter("dataset_flush_rate_hz").value)

        # State
        self.state_msg: Optional[Odometry] = None
        self.guidance_msg: Optional[Float64MultiArray] = None
        self.meta_msg: Optional[Float64MultiArray] = None
        self.base_cmd_msg: Optional[Float64MultiArray] = None
        self.base_debug_msg: Optional[Float64MultiArray] = None

        self.last_report_time_sec = 0.0
        self.last_delta_common = 0.0
        self.last_delta_diff = 0.0
        self.last_mode_code = -1
        self.run_start_sec = self.now_sec()

        # Policy
        self.policy = None
        if self.use_learned_policy:
            self.policy = _ResidualMLP(
                model_path=self.policy_path,
                policy_format=self.policy_format,
                input_dim=self.policy_input_dim,
                hidden_dim=self.policy_hidden_dim,
                device=self.policy_device,
            )

        # Dataset writer
        self.dataset_file = None
        self.dataset_writer = None
        if self.record_dataset:
            self._setup_dataset_writer()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Odometry, self.state_topic, self.state_callback, qos)
        self.create_subscription(Float64MultiArray, self.guidance_topic, self.guidance_callback, qos)
        self.create_subscription(Float64MultiArray, self.active_waypoint_meta_topic, self.meta_callback, qos)
        self.create_subscription(Float64MultiArray, self.base_thruster_cmd_topic, self.base_cmd_callback, qos)
        self.create_subscription(Float64MultiArray, self.base_controller_debug_topic, self.base_debug_callback, qos)

        self.cmd_pub = self.create_publisher(Float64MultiArray, self.thruster_cmd_topic, qos)
        self.debug_pub = self.create_publisher(Float64MultiArray, self.controller_debug_topic, qos)

        self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self.timer_callback)

        if self.record_dataset:
            self.create_timer(1.0 / max(self.dataset_flush_rate_hz, 1.0), self.dataset_flush_timer_callback)

        policy_desc = "heuristic"
        if self.use_learned_policy:
            if self.policy is not None and self.policy.available:
                policy_desc = f"learned:{self.policy_format}"
            else:
                policy_desc = f"fallback({self.policy.error if self.policy is not None else 'policy_none'})"

        self.get_logger().info(
            "controller_rl_residual started | mode=%s shadow=%s alpha=%.2f rate=%.0fHz "
            "cmd_raw=%s cmd_out=%s" % (
                policy_desc,
                self.shadow_mode,
                self.alpha_residual,
                self.publish_rate_hz,
                self.base_thruster_cmd_topic,
                self.thruster_cmd_topic,
            )
        )

    # ──────────────────────────────────────────────────────────────────────
    # Callbacks / helpers
    # ──────────────────────────────────────────────────────────────────────
    def state_callback(self, msg: Odometry) -> None:
        self.state_msg = msg

    def guidance_callback(self, msg: Float64MultiArray) -> None:
        self.guidance_msg = msg

    def meta_callback(self, msg: Float64MultiArray) -> None:
        self.meta_msg = msg

    def base_cmd_callback(self, msg: Float64MultiArray) -> None:
        self.base_cmd_msg = msg

    def base_debug_callback(self, msg: Float64MultiArray) -> None:
        self.base_debug_msg = msg

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def is_hold_mode(self, mode_code: int) -> bool:
        return mode_code in (0, 2)

    def clamp_thrust(self, value: float) -> float:
        lo = self.min_total_thrust * self.reverse_thrust_scale if value < 0.0 else self.min_total_thrust
        return max(lo, min(self.max_total_thrust, value))

    def clamp_abs(self, value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def rate_limit_residual(self, target: float, previous: float) -> float:
        delta = self.clamp_abs(target - previous, self.max_delta_residual_per_step)
        return previous + delta

    def current_state_tuple(self) -> Optional[Tuple[float, float, float, float, float, float]]:
        if self.state_msg is None:
            return None
        x = float(self.state_msg.pose.pose.position.x)
        y = float(self.state_msg.pose.pose.position.y)
        q = self.state_msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        vx = float(self.state_msg.twist.twist.linear.x)
        vy = float(self.state_msg.twist.twist.linear.y)
        speed = math.hypot(vx, vy)
        yaw_rate = float(self.state_msg.twist.twist.angular.z)
        return x, y, yaw, vx, vy, speed, yaw_rate

    def get_mode_and_meta(self) -> Tuple[int, float, float, float, float]:
        mode_code = 1
        pos_tol = 3.0
        hold_elapsed = 0.0
        hold_required = 0.0
        completed = 0.0
        if self.meta_msg is not None and len(self.meta_msg.data) >= 15:
            d = self.meta_msg.data
            mode_code = int(d[1])
            pos_tol = float(d[2])
            hold_elapsed = float(d[12])
            hold_required = float(d[13])
            completed = float(d[14])
        elif self.guidance_msg is not None and len(self.guidance_msg.data) >= 10:
            mode_code = int(self.guidance_msg.data[8])
            pos_tol = float(self.guidance_msg.data[9])
        return mode_code, pos_tol, hold_elapsed, hold_required, completed

    def extract_base_command(self) -> Optional[Tuple[float, float, float, float]]:
        if self.base_cmd_msg is None or len(self.base_cmd_msg.data) < 4:
            return None
        left_raw = float(self.base_cmd_msg.data[0])
        right_raw = float(self.base_cmd_msg.data[1])
        common_raw = 0.5 * (left_raw + right_raw)
        diff_raw = 0.5 * (right_raw - left_raw)
        return left_raw, right_raw, common_raw, diff_raw

    def build_observation(self) -> Optional[Tuple[list, Dict[str, Any]]]:
        state = self.current_state_tuple()
        base = self.extract_base_command()
        if state is None or base is None or self.guidance_msg is None or len(self.guidance_msg.data) < 12:
            return None

        x, y, yaw, vx, vy, speed, yaw_rate = state
        left_raw, right_raw, common_raw, diff_raw = base
        g = self.guidance_msg.data

        psi_ref = float(g[0])
        u_ref = float(g[1])
        e_ct = float(g[2])
        chi_p = float(g[3])
        dist_to_wp = float(g[4])
        heading_error = float(g[6])
        mode_code = int(g[8])
        pos_tol = float(g[9])

        hold_elapsed = 0.0
        hold_required = 0.0
        completed = 0.0
        seg_len = 0.0
        if self.meta_msg is not None and len(self.meta_msg.data) >= 15:
            m = self.meta_msg.data
            hold_elapsed = float(m[12])
            hold_required = float(m[13])
            completed = float(m[14])
            seg_dx = float(m[8]) - float(m[6])
            seg_dy = float(m[9]) - float(m[7])
            seg_len = math.hypot(seg_dx, seg_dy)

        yaw_error = wrap_angle(psi_ref - yaw)
        speed_error = u_ref - speed

        obs = [
            math.sin(yaw), math.cos(yaw),                               # 0-1 vehicle orientation
            math.sin(psi_ref), math.cos(psi_ref),                       # 2-3 reference LOS heading
            vx, vy, speed, yaw_rate,                                    # 4-7 velocity components
            e_ct, heading_error, yaw_error,                             # 8-10 tracking errors
            u_ref, speed_error, dist_to_wp,                             # 11-13 references and constraints
            float(mode_code), pos_tol,                                  # 14-15 operation mode context
            common_raw / max(self.max_total_thrust, 1.0),               # 16 raw common command
            diff_raw / max(self.max_total_thrust, 1.0),                 # 17 raw diff command
            math.sin(chi_p),                                            # 18 path geometry (sin)
            math.cos(chi_p),                                            # 19 path geometry (cos)
        ]

        aux = {
            "x": x, "y": y, "yaw": yaw, "vx": vx, "vy": vy, "speed": speed, "yaw_rate": yaw_rate,
            "psi_ref": psi_ref, "u_ref": u_ref, "e_ct": e_ct, "chi_p": chi_p,
            "dist_to_wp": dist_to_wp, "heading_error": heading_error, "yaw_error": yaw_error,
            "mode_code": mode_code, "pos_tol": pos_tol, "hold_elapsed": hold_elapsed,
            "hold_required": hold_required, "completed": completed, "seg_len": seg_len,
            "left_raw": left_raw, "right_raw": right_raw, "common_raw": common_raw, "diff_raw": diff_raw,
        }
        return obs, aux

    def heuristic_residual(self, aux: Dict[str, Any]) -> Tuple[float, float]:
        mode_code = int(aux["mode_code"])
        e_ct = float(aux["e_ct"])
        yaw_error = float(aux["yaw_error"])
        yaw_rate = float(aux["yaw_rate"])
        dist_to_wp = float(aux["dist_to_wp"])
        speed = float(aux["speed"])
        u_ref = float(aux["u_ref"])
        seg_len = float(aux["seg_len"])
        completed = float(aux["completed"])

        if completed >= 1.0:
            return 0.0, 0.0

        # Base heuristic: extra yaw authority for cte + heading residual,
        # and small common compensation for speed shortfall.
        delta_common = self.heuristic_k_speed_error * (u_ref - speed)
        delta_diff = (
            self.heuristic_k_cte * math.tanh(e_ct / 3.0)
            + self.heuristic_k_heading * yaw_error
            - self.heuristic_k_yaw_rate * yaw_rate
        )

        # Zigzag / short segments need slightly more aggressive differential action.
        if mode_code == 1 and 0.0 < seg_len < 18.0:
            delta_diff += self.heuristic_zigzag_gain * math.tanh(yaw_error / 0.35)

        # Finish / hold: brake common command near waypoint, damp differential.
        if mode_code == 2 and dist_to_wp < self.heuristic_finish_dist_thresh:
            delta_common -= self.heuristic_finish_brake_gain * (
                1.0 - max(0.0, dist_to_wp / max(self.heuristic_finish_dist_thresh, 1e-6))
            )
            delta_diff *= 0.70

        if mode_code == 0:
            delta_common *= 0.75
            delta_diff *= 0.80

        # Hold modes: be more conservative.
        if self.is_hold_mode(mode_code):
            delta_common = self.clamp_abs(delta_common, self.max_residual_common_hold)
            delta_diff = self.clamp_abs(delta_diff, self.max_residual_diff_hold)
        else:
            delta_common = self.clamp_abs(delta_common, self.max_residual_common_transit)
            delta_diff = self.clamp_abs(delta_diff, self.max_residual_diff_transit)

        return delta_common, delta_diff

    def predict_residual(self, obs: list, aux: Dict[str, Any]) -> Tuple[float, float, str]:
        mode_code = int(aux["mode_code"])
        seg_len   = float(aux.get("seg_len", 0.0))

        if not self.enable_residual:
            return 0.0, 0.0, "disabled"

        # Contextual alpha selector
        # Hold (start=0, finish=2): conservative to avoid disturbing WP capture
        # Transit (1): high authority to minimize CTE and sustain speed
        # Zigzag (transit + short segment): boost differential channel to correct
        # heading error on sharp turns without injecting excessive surge
        if self.is_hold_mode(mode_code):
            alpha_c = self.alpha_residual_hold
            alpha_d = self.alpha_residual_hold
        else:
            alpha_c = self.alpha_residual_transit
            # Boost differential authority on short segments (<20m, typical of zigzags)
            if 0.0 < seg_len < 20.0:
                alpha_d = self.alpha_residual_zigzag_diff
            else:
                alpha_d = self.alpha_residual_transit

        if self.use_learned_policy and self.policy is not None and self.policy.available:
            try:
                pred_common, pred_diff = self.policy.predict(obs)
                # Scale outputs from [-1, 1] approximately into physical residual bounds.
                if self.is_hold_mode(mode_code):
                    max_c = self.max_residual_common_hold
                    max_d = self.max_residual_diff_hold
                else:
                    max_c = self.max_residual_common_transit
                    max_d = self.max_residual_diff_transit

                delta_common = alpha_c * self.clamp_abs(pred_common, 1.0) * max_c
                delta_diff   = alpha_d * self.clamp_abs(pred_diff,   1.0) * max_d
                return delta_common, delta_diff, "learned"
            except Exception as exc:
                self.get_logger().warn(f"Policy inference failed, falling back to heuristic: {exc}")

        delta_common, delta_diff = self.heuristic_residual(aux)
        delta_common *= alpha_c
        delta_diff   *= alpha_d
        return delta_common, delta_diff, "heuristic"

    def apply_residual(self, aux: Dict[str, Any], delta_common_target: float, delta_diff_target: float) -> Dict[str, Any]:
        mode_code = int(aux["mode_code"])
        left_raw = float(aux["left_raw"])
        right_raw = float(aux["right_raw"])
        common_raw = float(aux["common_raw"])
        diff_raw = float(aux["diff_raw"])

        # Rate limiting on the residual itself
        delta_common = self.rate_limit_residual(delta_common_target, self.last_delta_common)
        delta_diff = self.rate_limit_residual(delta_diff_target, self.last_delta_diff)

        # Hard bounds
        if self.is_hold_mode(mode_code):
            delta_common = self.clamp_abs(delta_common, self.max_residual_common_hold)
            delta_diff = self.clamp_abs(delta_diff, self.max_residual_diff_hold)
        else:
            delta_common = self.clamp_abs(delta_common, self.max_residual_common_transit)
            delta_diff = self.clamp_abs(delta_diff, self.max_residual_diff_transit)

        if self.shadow_mode:
            delta_common = 0.0
            delta_diff = 0.0

        common_final = common_raw + delta_common
        diff_final = diff_raw + delta_diff

        left_final = self.clamp_thrust(common_final - diff_final)
        right_final = self.clamp_thrust(common_final + diff_final)
        common_final = 0.5 * (left_final + right_final)
        diff_final = 0.5 * (right_final - left_final)

        self.last_delta_common = delta_common
        self.last_delta_diff = delta_diff
        self.last_mode_code = mode_code

        return {
            "left_final": left_final,
            "right_final": right_final,
            "common_final": common_final,
            "diff_final": diff_final,
            "delta_common": delta_common,
            "delta_diff": delta_diff,
        }

    def publish_outputs(self, aux: Dict[str, Any], final_cmd: Dict[str, Any]) -> None:
        psi_ref = float(aux["psi_ref"])
        yaw = float(aux["yaw"])
        yaw_error = float(aux["yaw_error"])
        yaw_rate = float(aux["yaw_rate"])
        u_ref = float(aux["u_ref"])
        e_ct = float(aux["e_ct"])
        heading_error = float(aux["heading_error"])
        mode_code = int(aux["mode_code"])

        cmd = Float64MultiArray()
        cmd.data = [
            float(final_cmd["left_final"]),
            float(final_cmd["right_final"]),
            float(self.fixed_thruster_angle),
            float(self.fixed_thruster_angle),
        ]
        self.cmd_pub.publish(cmd)

        dbg = Float64MultiArray()
        dbg.data = [
            float(psi_ref),
            float(yaw),
            float(yaw_error),
            float(yaw_rate),
            float(u_ref),
            float(final_cmd["common_final"]),
            float(final_cmd["diff_final"]),
            float(final_cmd["left_final"]),
            float(final_cmd["right_final"]),
            float(e_ct),
            float(heading_error),
            float(mode_code),
        ]
        self.debug_pub.publish(dbg)

    def maybe_log_debug(self, aux: Dict[str, Any], final_cmd: Dict[str, Any], source: str) -> None:
        if not self.debug_enabled:
            return
        now = self.now_sec()
        if (now - self.last_report_time_sec) < max(self.debug_interval_sec, 0.05):
            return
        self.last_report_time_sec = now

        self.get_logger().info(
            "RL[m:%d:%s]|psi:%.2f/%.2f(e:%.2f,r:%.2f)|u:%.1f|c:%.1f,d:%.1f|raw:%.1f,%.1f|res:%.1f,%.1f|th:%.1f,%.1f|e_ct:%.1f" % (
                int(aux["mode_code"]),
                source[:3],
                float(aux["psi_ref"]),
                float(aux["yaw"]),
                float(aux["yaw_error"]),
                float(aux["yaw_rate"]),
                float(aux["u_ref"]),
                float(final_cmd["common_final"]),
                float(final_cmd["diff_final"]),
                float(aux["left_raw"]),
                float(aux["right_raw"]),
                float(final_cmd["delta_common"]),
                float(final_cmd["delta_diff"]),
                float(final_cmd["left_final"]),
                float(final_cmd["right_final"]),
                float(aux["e_ct"]),
            )
        )

    # ──────────────────────────────────────────────────────────────────────
    # Dataset
    # ──────────────────────────────────────────────────────────────────────
    def _resolve_output_dirs(self, output_root: str) -> str:
        candidates = [
            os.path.abspath(output_root),
            os.path.join(os.path.expanduser("~"), output_root),
        ]
        base = None
        for candidate in candidates:
            try:
                os.makedirs(candidate, exist_ok=True)
                test_file = os.path.join(candidate, ".write_test")
                with open(test_file, "w", encoding="utf-8") as _:
                    pass
                os.remove(test_file)
                base = candidate
                break
            except OSError:
                continue

        if base is None:
            base = os.path.join("/tmp", "vrx_rl_dataset")
            os.makedirs(base, exist_ok=True)
            self.get_logger().warn(f"[rl_dataset] fallback output_root={base}")

        raw_dir = os.path.join(base, "raw")
        os.makedirs(raw_dir, exist_ok=True)
        return raw_dir

    def _setup_dataset_writer(self) -> None:
        raw_dir = self._resolve_output_dirs(self.dataset_output_root)
        tag = self.dataset_tag if self.dataset_tag else "mppi_rl"
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(raw_dir, f"{tag}_{stamp}_dataset.csv")
        self.dataset_file = open(path, "w", newline="", buffering=1, encoding="utf-8")
        self.dataset_writer = csv.writer(self.dataset_file)
        self.dataset_writer.writerow([
            "t", "mode_code", "shadow_mode", "policy_source",
            "x", "y", "yaw", "vx", "vy", "speed", "yaw_rate",
            "psi_ref", "u_ref", "e_ct", "chi_p", "dist_to_wp", "heading_error",
            "pos_tol", "hold_elapsed", "hold_required", "completed", "seg_len",
            "left_raw", "right_raw", "common_raw", "diff_raw",
            "delta_common", "delta_diff", "left_final", "right_final",
        ])
        self.get_logger().info(f"[rl_dataset] writing → {path}")

    def write_dataset_row(self, aux: Dict[str, Any], final_cmd: Dict[str, Any], source: str) -> None:
        if not self.record_dataset or self.dataset_writer is None:
            return
        t = self.now_sec() - self.run_start_sec
        self.dataset_writer.writerow([
            f"{t:.3f}",
            int(aux["mode_code"]),
            int(self.shadow_mode),
            source,
            f"{float(aux['x']):.4f}",
            f"{float(aux['y']):.4f}",
            f"{float(aux['yaw']):.4f}",
            f"{float(aux['vx']):.4f}",
            f"{float(aux['vy']):.4f}",
            f"{float(aux['speed']):.4f}",
            f"{float(aux['yaw_rate']):.4f}",
            f"{float(aux['psi_ref']):.4f}",
            f"{float(aux['u_ref']):.4f}",
            f"{float(aux['e_ct']):.4f}",
            f"{float(aux['chi_p']):.4f}",
            f"{float(aux['dist_to_wp']):.4f}",
            f"{float(aux['heading_error']):.4f}",
            f"{float(aux['pos_tol']):.4f}",
            f"{float(aux['hold_elapsed']):.4f}",
            f"{float(aux['hold_required']):.4f}",
            f"{float(aux['completed']):.0f}",
            f"{float(aux['seg_len']):.4f}",
            f"{float(aux['left_raw']):.4f}",
            f"{float(aux['right_raw']):.4f}",
            f"{float(aux['common_raw']):.4f}",
            f"{float(aux['diff_raw']):.4f}",
            f"{float(final_cmd['delta_common']):.4f}",
            f"{float(final_cmd['delta_diff']):.4f}",
            f"{float(final_cmd['left_final']):.4f}",
            f"{float(final_cmd['right_final']):.4f}",
        ])

    def dataset_flush_timer_callback(self) -> None:
        if self.dataset_file is not None:
            self.dataset_file.flush()
            os.fsync(self.dataset_file.fileno())

    # ──────────────────────────────────────────────────────────────────────
    # Main loop
    # ──────────────────────────────────────────────────────────────────────
    def timer_callback(self) -> None:
        built = self.build_observation()
        if built is None:
            return
        obs, aux = built

        if int(aux["completed"]) == 1:
            final_cmd = {
                "left_final": 0.0,
                "right_final": 0.0,
                "common_final": 0.0,
                "diff_final": 0.0,
                "delta_common": 0.0,
                "delta_diff": 0.0,
            }
            self.publish_outputs(aux, final_cmd)
            self.write_dataset_row(aux, final_cmd, "done")
            self.maybe_log_debug(aux, final_cmd, "done")
            return

        delta_common_target, delta_diff_target, source = self.predict_residual(obs, aux)
        final_cmd = self.apply_residual(aux, delta_common_target, delta_diff_target)
        self.publish_outputs(aux, final_cmd)
        self.write_dataset_row(aux, final_cmd, source)
        self.maybe_log_debug(aux, final_cmd, source)

    def shutdown(self) -> None:
        if self.dataset_file is not None:
            self.dataset_file.flush()
            os.fsync(self.dataset_file.fileno())
            self.dataset_file.close()
            self.get_logger().info("[rl_dataset] dataset CSV closed.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerRLResidual()
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