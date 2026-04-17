#!/usr/bin/env python3

from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


# Node for issuing physical constraints and commands to thrusters
class ThrusterCommander(Node):
    # Node initialization
    def __init__(self) -> None:
        super().__init__("thruster_commander")

        self.declare_parameter("thruster_cmd_topic", "/wamv/control/thruster_cmd")
        self.declare_parameter("left_thrust_topic", "/wamv/thrusters/left/thrust")
        self.declare_parameter("right_thrust_topic", "/wamv/thrusters/right/thrust")
        self.declare_parameter("left_pos_topic", "/wamv/thrusters/left/pos")
        self.declare_parameter("right_pos_topic", "/wamv/thrusters/right/pos")

        self.declare_parameter("send_positions", True)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("thrust_min", -750.0)
        self.declare_parameter("thrust_max", 750.0)
        self.declare_parameter("pos_min", -1.57079632679)
        self.declare_parameter("pos_max", 1.57079632679)
        self.declare_parameter("debug_enabled", True)
        self.declare_parameter("debug_interval_sec", 1.0)

        self.thruster_cmd_topic = str(self.get_parameter("thruster_cmd_topic").value)
        self.left_thrust_topic = str(self.get_parameter("left_thrust_topic").value)
        self.right_thrust_topic = str(self.get_parameter("right_thrust_topic").value)
        self.left_pos_topic = str(self.get_parameter("left_pos_topic").value)
        self.right_pos_topic = str(self.get_parameter("right_pos_topic").value)

        self.send_positions = bool(self.get_parameter("send_positions").value)
        self.command_timeout_sec = float(self.get_parameter("command_timeout_sec").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.thrust_min = float(self.get_parameter("thrust_min").value)
        self.thrust_max = float(self.get_parameter("thrust_max").value)
        self.pos_min = float(self.get_parameter("pos_min").value)
        self.pos_max = float(self.get_parameter("pos_max").value)
        self.debug_enabled = bool(self.get_parameter("debug_enabled").value)
        self.debug_interval_sec = float(self.get_parameter("debug_interval_sec").value)

        self.last_cmd: Optional[Float64MultiArray] = None
        self.last_cmd_time_sec: Optional[float] = None
        self.last_report_time_sec: float = 0.0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Float64MultiArray, self.thruster_cmd_topic, self.cmd_callback, qos)

        self.left_thrust_pub = self.create_publisher(Float64, self.left_thrust_topic, qos)
        self.right_thrust_pub = self.create_publisher(Float64, self.right_thrust_topic, qos)
        self.left_pos_pub = self.create_publisher(Float64, self.left_pos_topic, qos)
        self.right_pos_pub = self.create_publisher(Float64, self.right_pos_topic, qos)

        self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self.timer_callback)

        self.get_logger().info("thruster_commander started")

    # Calculate current ROS time in seconds
    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # Constrain value within boundaries
    def clamp(self, value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    # Cache latest thruster commands
    def cmd_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 4:
            self.get_logger().warn("Received thruster_cmd with insufficient fields")
            return
        self.last_cmd = msg
        self.last_cmd_time_sec = self.now_sec()

    # Fast float publisher helper
    def publish_float(self, pub, value: float) -> None:
        m = Float64()
        m.data = float(value)
        pub.publish(m)

    # Log command state summary
    def maybe_log_debug(self, left: float, right: float, left_pos: float, right_pos: float, timed_out: bool) -> None:
        if not self.debug_enabled:
            return
        now = self.now_sec()
        if (now - self.last_report_time_sec) < max(self.debug_interval_sec, 0.05):
            return
        self.last_report_time_sec = now
        self.get_logger().info("CMD|L:%.1f/pos:%.2f R:%.1f/pos:%.2f|to:%s" % (left, left_pos, right, right_pos, str(timed_out)[0]))

    # Enforce timeouts and transmit clamped thrust continuously
    def timer_callback(self) -> None:
        left = 0.0
        right = 0.0
        left_pos = 0.0
        right_pos = 0.0
        timed_out = True

        if self.last_cmd is not None and self.last_cmd_time_sec is not None:
            if (self.now_sec() - self.last_cmd_time_sec) <= self.command_timeout_sec:
                timed_out = False
                left = float(self.last_cmd.data[0])
                right = float(self.last_cmd.data[1])
                left_pos = float(self.last_cmd.data[2])
                right_pos = float(self.last_cmd.data[3])

        left = self.clamp(left, self.thrust_min, self.thrust_max)
        right = self.clamp(right, self.thrust_min, self.thrust_max)
        left_pos = self.clamp(left_pos, self.pos_min, self.pos_max)
        right_pos = self.clamp(right_pos, self.pos_min, self.pos_max)

        self.publish_float(self.left_thrust_pub, left)
        self.publish_float(self.right_thrust_pub, right)

        if self.send_positions:
            self.publish_float(self.left_pos_pub, left_pos)
            self.publish_float(self.right_pos_pub, right_pos)

        self.maybe_log_debug(left, right, left_pos, right_pos, timed_out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThrusterCommander()
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