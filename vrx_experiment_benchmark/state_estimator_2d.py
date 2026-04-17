#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64MultiArray

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)

from vrx_experiment_benchmark.route_utils import (
    geodetic_to_local_xy,
    quaternion_to_yaw,
    unwrap_angle,
    wrap_angle,
)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


# Sensor fusion node emitting smoothed Odometry
class StateEstimator2D(Node):
    def __init__(self) -> None:
        super().__init__("state_estimator_2d")

        self.declare_parameter("gps_topic", "/wamv/sensors/gps/gps/fix")
        self.declare_parameter("imu_topic", "/wamv/sensors/imu/imu/data")
        self.declare_parameter("output_topic", "/wamv/navigation/state2d")
        self.declare_parameter("origin_topic", "/wamv/navigation/local_origin")
        self.declare_parameter("debug_topic", "/wamv/navigation/debug_state2d")

        self.declare_parameter("local_frame_id", "map_local")
        self.declare_parameter("child_frame_id", "wamv/base_link_est")

        self.declare_parameter("use_first_gps_as_origin", True)
        self.declare_parameter("fixed_origin_lat", 0.0)
        self.declare_parameter("fixed_origin_lon", 0.0)

        # Light position filtering for high-frequency RTK GPS
        self.declare_parameter("position_alpha", 0.85)
        self.declare_parameter("velocity_alpha", 0.20)
        self.declare_parameter("yaw_alpha", 0.12)

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("origin_publish_rate_hz", 1.0)
        self.declare_parameter("min_dt_for_velocity", 0.02)
        self.declare_parameter("min_speed_for_course_yaw", 0.30)

        # Configurable covariances
        self.declare_parameter("pose_cov_x", 0.05)
        self.declare_parameter("pose_cov_y", 0.05)
        self.declare_parameter("pose_cov_yaw", 0.20)
        self.declare_parameter("twist_cov_vx", 0.10)
        self.declare_parameter("twist_cov_vy", 0.10)
        self.declare_parameter("twist_cov_yaw_rate", 0.05)

        # Debug timeout warnings
        self.declare_parameter("warn_if_no_gps_after_sec", 3.0)

        self.gps_topic = str(self.get_parameter("gps_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.origin_topic = str(self.get_parameter("origin_topic").value)
        self.debug_topic = str(self.get_parameter("debug_topic").value)

        self.local_frame_id = str(self.get_parameter("local_frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)

        self.use_first_gps_as_origin = bool(self.get_parameter("use_first_gps_as_origin").value)
        self.fixed_origin_lat = float(self.get_parameter("fixed_origin_lat").value)
        self.fixed_origin_lon = float(self.get_parameter("fixed_origin_lon").value)

        self.position_alpha = float(self.get_parameter("position_alpha").value)
        self.velocity_alpha = float(self.get_parameter("velocity_alpha").value)
        self.yaw_alpha = float(self.get_parameter("yaw_alpha").value)

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.origin_publish_rate_hz = float(self.get_parameter("origin_publish_rate_hz").value)
        self.min_dt_for_velocity = float(self.get_parameter("min_dt_for_velocity").value)
        self.min_speed_for_course_yaw = float(self.get_parameter("min_speed_for_course_yaw").value)

        self.pose_cov_x = float(self.get_parameter("pose_cov_x").value)
        self.pose_cov_y = float(self.get_parameter("pose_cov_y").value)
        self.pose_cov_yaw = float(self.get_parameter("pose_cov_yaw").value)
        self.twist_cov_vx = float(self.get_parameter("twist_cov_vx").value)
        self.twist_cov_vy = float(self.get_parameter("twist_cov_vy").value)
        self.twist_cov_yaw_rate = float(self.get_parameter("twist_cov_yaw_rate").value)

        self.warn_if_no_gps_after_sec = float(self.get_parameter("warn_if_no_gps_after_sec").value)

        self.origin_lat: Optional[float] = None
        self.origin_lon: Optional[float] = None

        self.origin_locked = False
        self.has_received_gps = False
        self.has_received_imu = False
        self.warned_no_gps = False

        if not self.use_first_gps_as_origin:
            self.origin_lat = self.fixed_origin_lat
            self.origin_lon = self.fixed_origin_lon
            self.origin_locked = True
            self.get_logger().info(
                f"Using fixed local origin: lat={self.origin_lat:.10f}, lon={self.origin_lon:.10f}"
            )
        else:
            self.get_logger().info("Using first GPS fix as dynamic local origin")

        self.latest_stamp = self.get_clock().now().to_msg()
        self.start_time_sec = self.get_clock().now().nanoseconds * 1e-9

        self.raw_x: Optional[float] = None
        self.raw_y: Optional[float] = None

        self.filt_x: Optional[float] = None
        self.filt_y: Optional[float] = None

        self.prev_filt_x: Optional[float] = None
        self.prev_filt_y: Optional[float] = None
        self.prev_gps_time: Optional[float] = None

        self.vx: float = 0.0
        self.vy: float = 0.0
        self.speed_over_ground: float = 0.0

        self.yaw_raw: Optional[float] = None
        self.yaw_unwrapped: Optional[float] = None
        self.yaw_filtered: Optional[float] = None
        self.yaw_rate: float = 0.0
        self.course_yaw: Optional[float] = None

        self.last_gps_msg: Optional[NavSatFix] = None
        self.last_imu_msg: Optional[Imu] = None

        sensor_qos = qos_profile_sensor_data

        origin_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        debug_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(NavSatFix, self.gps_topic, self.gps_callback, sensor_qos)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, sensor_qos)

        self.pub = self.create_publisher(Odometry, self.output_topic, odom_qos)
        self.origin_pub = self.create_publisher(Float64MultiArray, self.origin_topic, origin_qos)
        self.debug_pub = self.create_publisher(Float64MultiArray, self.debug_topic, debug_qos)

        timer_period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.create_timer(timer_period, self.publish_state)

        origin_timer_period = 1.0 / max(self.origin_publish_rate_hz, 0.1)
        self.create_timer(origin_timer_period, self.publish_origin)

        self.create_timer(0.5, self.watchdog_status)

        self.get_logger().info("state_estimator_2d started")

    # Check for hardware or sim sensor connectivity errors
    def watchdog_status(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now_sec - self.start_time_sec

        if not self.has_received_gps and elapsed >= self.warn_if_no_gps_after_sec and not self.warned_no_gps:
            self.get_logger().warn(
                f"No GPS received yet on {self.gps_topic} after {elapsed:.1f} s"
            )
            self.warned_no_gps = True

    # Update absolute position via GPS
    def gps_callback(self, msg: NavSatFix) -> None:
        self.last_gps_msg = msg
        self.has_received_gps = True

        lat = float(msg.latitude)
        lon = float(msg.longitude)

        if self.use_first_gps_as_origin and not self.origin_locked:
            self.origin_lat = lat
            self.origin_lon = lon
            self.origin_locked = True
            self.get_logger().info(
                f"Locked local origin from first GPS fix: "
                f"lat={self.origin_lat:.10f}, lon={self.origin_lon:.10f}"
            )

        if self.origin_lat is None or self.origin_lon is None:
            return

        x, y = geodetic_to_local_xy(lat, lon, self.origin_lat, self.origin_lon)
        self.raw_x = x
        self.raw_y = y
        self.latest_stamp = msg.header.stamp

        current_time = stamp_to_seconds(msg.header.stamp)

        if self.filt_x is None or self.filt_y is None:
            self.filt_x = x
            self.filt_y = y
            self.prev_filt_x = x
            self.prev_filt_y = y
            self.prev_gps_time = current_time
            return

        self.filt_x = self.position_alpha * x + (1.0 - self.position_alpha) * self.filt_x
        self.filt_y = self.position_alpha * y + (1.0 - self.position_alpha) * self.filt_y

        if self.prev_gps_time is not None:
            dt = current_time - self.prev_gps_time
            if (
                dt >= self.min_dt_for_velocity
                and self.prev_filt_x is not None
                and self.prev_filt_y is not None
            ):
                vx_meas = (self.filt_x - self.prev_filt_x) / dt
                vy_meas = (self.filt_y - self.prev_filt_y) / dt

                self.vx = self.velocity_alpha * vx_meas + (1.0 - self.velocity_alpha) * self.vx
                self.vy = self.velocity_alpha * vy_meas + (1.0 - self.velocity_alpha) * self.vy
                self.speed_over_ground = math.hypot(self.vx, self.vy)

                if self.speed_over_ground >= self.min_speed_for_course_yaw:
                    self.course_yaw = math.atan2(self.vy, self.vx)

        self.prev_filt_x = self.filt_x
        self.prev_filt_y = self.filt_y
        self.prev_gps_time = current_time

    # Update orientation and angular rate via IMU
    def imu_callback(self, msg: Imu) -> None:
        self.last_imu_msg = msg
        self.has_received_imu = True

        q = msg.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.yaw_raw = yaw
        self.yaw_rate = float(msg.angular_velocity.z)
        self.latest_stamp = msg.header.stamp

        if self.yaw_unwrapped is None:
            self.yaw_unwrapped = yaw
            self.yaw_filtered = yaw
            return

        self.yaw_unwrapped = unwrap_angle(yaw, self.yaw_unwrapped)
        self.yaw_filtered = (
            self.yaw_alpha * self.yaw_unwrapped
            + (1.0 - self.yaw_alpha) * self.yaw_filtered
        )

    # Broadcast reference coordinates framing the local origin
    def publish_origin(self) -> None:
        if self.origin_lat is None or self.origin_lon is None:
            return

        msg = Float64MultiArray()
        msg.data = [float(self.origin_lat), float(self.origin_lon)]
        self.origin_pub.publish(msg)

    # Publish raw sensor versus filtered estimation streams
    def publish_debug(self) -> None:
        if self.filt_x is None or self.filt_y is None or self.yaw_filtered is None:
            return

        msg = Float64MultiArray()
        course = self.course_yaw if self.course_yaw is not None else float("nan")
        msg.data = [
            float(self.raw_x) if self.raw_x is not None else float("nan"),
            float(self.raw_y) if self.raw_y is not None else float("nan"),
            float(self.filt_x),
            float(self.filt_y),
            float(self.vx),
            float(self.vy),
            float(self.speed_over_ground),
            float(self.yaw_raw) if self.yaw_raw is not None else float("nan"),
            float(self.yaw_filtered),
            float(self.yaw_rate),
            float(course),
            float(self.origin_lat) if self.origin_lat is not None else float("nan"),
            float(self.origin_lon) if self.origin_lon is not None else float("nan"),
        ]
        self.debug_pub.publish(msg)

    # Transmit final synthesized state message
    def publish_state(self) -> None:
        if self.filt_x is None or self.filt_y is None or self.yaw_filtered is None:
            return

        msg = Odometry()
        msg.header.stamp = self.latest_stamp
        msg.header.frame_id = self.local_frame_id
        msg.child_frame_id = self.child_frame_id

        msg.pose.pose.position.x = float(self.filt_x)
        msg.pose.pose.position.y = float(self.filt_y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(wrap_angle(self.yaw_filtered))

        msg.twist.twist.linear.x = float(self.vx)
        msg.twist.twist.linear.y = float(self.vy)
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.z = float(self.yaw_rate)

        msg.pose.covariance[0] = self.pose_cov_x
        msg.pose.covariance[7] = self.pose_cov_y
        msg.pose.covariance[35] = self.pose_cov_yaw

        msg.twist.covariance[0] = self.twist_cov_vx
        msg.twist.covariance[7] = self.twist_cov_vy
        msg.twist.covariance[35] = self.twist_cov_yaw_rate

        self.pub.publish(msg)
        self.publish_debug()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StateEstimator2D()
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