#!/usr/bin/env python3

import math

import rclpy

from rclpy.node import Node

from std_msgs.msg import Int64MultiArray

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Quaternion



from tf_transformations import quaternion_from_euler


class WheelOdometry(Node):

    def __init__(self):

        super().__init__("wheel_odometry")

        self.get_logger().info(
            "Wheel Odometry Node Started"
        )

        # ----------------------------------
        # Robot Parameters
        # ----------------------------------

        self.declare_parameter(
            "wheel_radius",
            0.0585
        )

        self.declare_parameter(
            "wheel_base",
            0.535
        )

        self.declare_parameter(
            "encoder_cpr",
            46367   # can try with diff 
        )

        self.wheel_radius = (
            self.get_parameter(
                "wheel_radius"
            ).value
        )

        self.wheel_base = (
            self.get_parameter(
                "wheel_base"
            ).value
        )

        self.encoder_cpr = (
            self.get_parameter(
                "encoder_cpr"
            ).value
        )

        # ----------------------------------
        # Publisher
        # ----------------------------------

        self.odom_pub = self.create_publisher(
            Odometry,
            "/wheel/odom",
            10
        )

        

        # ----------------------------------
        # Encoder Subscriber
        # ----------------------------------

        self.encoder_sub = (
            self.create_subscription(
                Int64MultiArray,
                "/encoder_ticks",
                self.encoder_callback,
                10
            )
        )

        # ----------------------------------
        # Robot Pose
        # ----------------------------------

        self.x = 0.0

        self.y = 0.0

        self.theta = 0.0

        # ----------------------------------
        # Previous Encoder Values
        # ----------------------------------

        self.prev_left_ticks = None

        self.prev_right_ticks = None

        # ----------------------------------
        # Current Encoder Values
        # ----------------------------------

        self.left_ticks = 0

        self.right_ticks = 0

        # ----------------------------------
        # Time
        # ----------------------------------

        self.prev_time = (
            self.get_clock().now()
        )

        self.get_logger().info(
            "Waiting for encoder data..."
        )

    

    # ======================================
    # Encoder Callback
    # ======================================

    def encoder_callback(self, msg):

        self.left_ticks = msg.data[0]
        self.right_ticks = msg.data[1]

        # First encoder message
        if self.prev_left_ticks is None:

            self.prev_left_ticks = self.left_ticks
            self.prev_right_ticks = self.right_ticks
            self.prev_time = self.get_clock().now()

            return

        # -----------------------------
        # Time Difference
        # -----------------------------

        current_time = self.get_clock().now()

        dt = (
            current_time - self.prev_time
        ).nanoseconds / 1e9

        if dt <= 0.0:
            return

        # -----------------------------
        # Tick Difference
        # -----------------------------

        delta_left_ticks = (
            self.left_ticks -
            self.prev_left_ticks
        )

        delta_right_ticks = (
            self.right_ticks -
            self.prev_right_ticks
        )
        
        if abs(delta_left_ticks) < 1:
            delta_left_ticks = 0

        if abs(delta_right_ticks) < 1:
            delta_right_ticks = 0


        # self.get_logger().info(
        #     f"ΔL={delta_left_ticks} ΔR={delta_right_ticks}"
        # )
        # self.get_logger().info(
        #     f"x={self.x:.3f} y={self.y:.3f} theta={self.theta:.3f}"
        # )

        # -----------------------------
        # Save Current Values
        # -----------------------------

        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks
        self.prev_time = current_time

        # -----------------------------
        # Distance Per Tick
        # -----------------------------

        distance_per_tick = (
            2.0 *
            math.pi *
            self.wheel_radius
        ) / self.encoder_cpr

        # -----------------------------
        # Wheel Distance
        # -----------------------------

        left_distance = (
            delta_left_ticks *
            distance_per_tick
        )

        right_distance = (
            delta_right_ticks *
            distance_per_tick
        )

        # -----------------------------
        # Robot Motion
        # -----------------------------

        linear_distance = (
            left_distance +
            right_distance
        ) / 2.0

        angular_distance = (
            right_distance -
            left_distance
        ) / self.wheel_base

        # -----------------------------
        # Update Pose
        # -----------------------------

        delta_theta = angular_distance

        self.x += linear_distance * math.cos(
            self.theta + delta_theta / 2.0
        )

        self.y += linear_distance * math.sin(
            self.theta + delta_theta / 2.0
        )

        self.theta += delta_theta

        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta)
        )

        # -----------------------------
        # Robot Velocity
        # -----------------------------

        linear_velocity = (
            linear_distance /
            dt
        )

        angular_velocity = (
            angular_distance /
            dt
        )

        # Publish everything
        self.publish_odometry(
            current_time,
            linear_velocity,
            angular_velocity
        )

        # ======================================
    # Publish Odometry
    # ======================================

    def publish_odometry(
            self,
            current_time,
            linear_velocity,
            angular_velocity
        ):

        # -----------------------------
        # Quaternion
        # -----------------------------

        qx, qy, qz, qw = quaternion_from_euler(
            0.0,
            0.0,
            self.theta
        )

        # -----------------------------
        # Odometry Message
        # -----------------------------

        odom = Odometry()

        odom.header.stamp = current_time.to_msg()

        odom.header.frame_id = "odom"

        odom.child_frame_id = "base_footprint"

        # Position

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Velocity

        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = angular_velocity

        # -----------------------------
        # Pose Covariance
        # -----------------------------

        odom.pose.covariance = [

            0.05,0,0,0,0,0,
            0,0.05,0,0,0,0,
            0,0,99999,0,0,0,
            0,0,0,99999,0,0,
            0,0,0,0,99999,0,
            0,0,0,0,0,0.08

        ]

        # -----------------------------
        # Twist Covariance
        # -----------------------------

        odom.twist.covariance = [

            0.05,0,0,0,0,0,
            0,0.02,0,0,0,0,
            0,0,99999,0,0,0,
            0,0,0,99999,0,0,
            0,0,0,0,99999,0,
            0,0,0,0,0,0.08

        ]

        # -----------------------------
        # Publish
        # -----------------------------

        self.odom_pub.publish(
            odom
        )
def main(args=None):

    rclpy.init(args=args)

    node = WheelOdometry()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()