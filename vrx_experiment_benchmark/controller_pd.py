#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from vrx_experiment_benchmark.route_utils import wrap_angle, quaternion_to_yaw, MODE_TO_CODE


# Proportional-Derivative heading controller with surge feed-forward
class ControllerPD(Node):
    # Node initialization
    def __init__(self) -> None:
        super().__init__("controller_pd")

        self.declare_parameter("state_topic", "/wamv/navigation/state2d")
        self.declare_parameter("guidance_topic", "/wamv/control/guidance_cmd")
        self.declare_parameter("thruster_cmd_topic", "/wamv/control/thruster_cmd")
        self.declare_parameter("controller_debug_topic", "/wamv/control/controller_debug")
        self.declare_parameter("publish_rate_hz", 20.0)

        self.declare_parameter("kp_yaw", 2.8)
        self.declare_parameter("kd_yaw", 0.9)
        self.declare_parameter("surge_gain", 3.0)
        self.declare_parameter("yaw_to_diff_gain", 2.0)
        self.declare_parameter("max_thrust", 750.0)
        self.declare_parameter("fixed_thruster_angle", 0.0)
        self.declare_parameter("debug_enabled", True)
        self.declare_parameter("debug_interval_sec", 1.0)

        self.state_topic = str(self.get_parameter("state_topic").value)
        self.guidance_topic = str(self.get_parameter("guidance_topic").value)
        self.thruster_cmd_topic = str(self.get_parameter("thruster_cmd_topic").value)
        self.controller_debug_topic = str(self.get_parameter("controller_debug_topic").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.kp_yaw = float(self.get_parameter("kp_yaw").value)
        self.kd_yaw = float(self.get_parameter("kd_yaw").value)
        self.surge_gain = float(self.get_parameter("surge_gain").value)
        self.yaw_to_diff_gain = float(self.get_parameter("yaw_to_diff_gain").value)
        self.max_thrust = float(self.get_parameter("max_thrust").value)
        self.fixed_thruster_angle = float(self.get_parameter("fixed_thruster_angle").value)
        self.debug_enabled = bool(self.get_parameter("debug_enabled").value)
        self.debug_interval_sec = float(self.get_parameter("debug_interval_sec").value)

        self.state_msg: Optional[Odometry] = None
        self.guidance_msg: Optional[Float64MultiArray] = None
        self.last_report_time_sec: float = 0.0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Odometry, self.state_topic, self.state_callback, qos)
        self.create_subscription(Float64MultiArray, self.guidance_topic, self.guidance_callback, qos)

        self.cmd_pub = self.create_publisher(Float64MultiArray, self.thruster_cmd_topic, qos)
        self.debug_pub = self.create_publisher(Float64MultiArray, self.controller_debug_topic, qos)

        self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self.timer_callback)

        self.get_logger().info("controller_pd started")

    # Caches vehicle state
    def state_callback(self, msg: Odometry) -> None:
        self.state_msg = msg

    # Caches guidance logic values
    def guidance_callback(self, msg: Float64MultiArray) -> None:
        self.guidance_msg = msg

    # Current ROS time in seconds
    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # Limit symmetric thruster output
    def clamp(self, value: float) -> float:
        return max(-self.max_thrust, min(self.max_thrust, value))

    # Log control logic metrics
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
        saturated_left: bool,
        saturated_right: bool,
    ) -> None:
        if not self.debug_enabled:
            return

        now = self.now_sec()
        if (now - self.last_report_time_sec) < max(self.debug_interval_sec, 0.05):
            return
        self.last_report_time_sec = now

        self.get_logger().info(
            "PD[m:%d]|psi:%.2f/%.2f(e:%.2f,r:%.2f)|u:%.1f|c:%.1f,d:%.1f|th:%.1f(%s),%.1f(%s)|e_ct:%.1f" % (
                mode_code, psi_ref, yaw, yaw_error, yaw_rate, u_ref, common, differential, left_thrust, str(saturated_left)[0], right_thrust, str(saturated_right)[0], e_ct
            )
        )

    # Process loop for PD control output
    def timer_callback(self) -> None:
        if self.state_msg is None or self.guidance_msg is None:
            return

        g = self.guidance_msg.data
        if len(g) < 12:
            return

        psi_ref = float(g[0])
        u_ref = float(g[1])
        e_ct = float(g[2])
        heading_error_from_guidance = float(g[6])
        mode_code = int(g[8])

        q = self.state_msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        yaw_rate = float(self.state_msg.twist.twist.angular.z)

        yaw_error = wrap_angle(psi_ref - yaw)

        common = self.surge_gain * u_ref
        yaw_effort = self.kp_yaw * yaw_error - self.kd_yaw * yaw_rate
        differential = self.yaw_to_diff_gain * yaw_effort

        if mode_code in (MODE_TO_CODE["start"], MODE_TO_CODE["finish"]):
            common *= 0.7

        raw_left = common - differential
        raw_right = common + differential
        left_thrust = self.clamp(raw_left)
        right_thrust = self.clamp(raw_right)
        saturated_left = not math.isclose(left_thrust, raw_left, rel_tol=0.0, abs_tol=1e-9)
        saturated_right = not math.isclose(right_thrust, raw_right, rel_tol=0.0, abs_tol=1e-9)

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
            saturated_left,
            saturated_right,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerPD()
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