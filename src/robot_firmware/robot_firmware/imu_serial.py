#!/usr/bin/env python3

import time
import serial
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

from transforms3d.euler import euler2quat


class ImuSerialNode(Node):

    def __init__(self):
        super().__init__('imu_serial')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        try:
            self.ser = serial.Serial(
                port,
                baud,
                timeout=0.05
            )

            time.sleep(2)

            self.ser.reset_input_buffer()

            self.get_logger().info(
                f'Connected to {port}'
            )

        except Exception as e:
            self.get_logger().error(
                f'Failed to open serial port: {e}'
            )
            raise

        self.imu_pub = self.create_publisher(
            Imu,
            '/imu/data',
            10
        )
        self._serial_buf = b""
        self.timer = self.create_timer(
            0.02,   # 50 Hz
            self.read_serial
        )

    def read_serial(self):
        try:
            n = self.ser.in_waiting
            if n:
                self._serial_buf += self.ser.read(n)

            while b"\n" in self._serial_buf:
                raw, self._serial_buf = self._serial_buf.split(b"\n", 1)
                line = raw.decode('utf-8', errors='ignore').strip()

                if not line.startswith('I,'):
                    continue

                data = line.split(',')
                if len(data) < 10:
                    continue

                yaw_deg = float(data[1])
                pitch_deg = float(data[2])
                roll_deg = float(data[3])
                gx = float(data[4])
                gy = float(data[5])
                gz = float(data[6])
                ax = float(data[7])
                ay = float(data[8])
                az = float(data[9])

                # ========================================================
                # APPLY PITCH FLIP (180 DEGREES) TO INVERT YAW DIRECTION
                # ========================================================
                # Adding 180 degrees to pitch flips the IMU upside down.
                # Because it's flipped upside down, the Yaw orientation
                # and its angular velocity Z direction must be inverted.
                pitch_deg = pitch_deg + 180.0
                yaw_deg = -yaw_deg   # Reverses the absolute orientation angle
                gz = -gz             # Reverses the raw spinning speed rate
                # ========================================================

                roll = math.radians(roll_deg)
                pitch = math.radians(pitch_deg)
                yaw = math.radians(yaw_deg)
                qw, qx, qy, qz = euler2quat(roll, pitch, yaw)

                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "imu_link"
                msg.orientation.x = qx
                msg.orientation.y = qy
                msg.orientation.z = qz
                msg.orientation.w = qw
                msg.angular_velocity.x = gx
                msg.angular_velocity.y = gy
                msg.angular_velocity.z = gz  # Now correctly publishes reversed speed
                msg.linear_acceleration.x = ax
                msg.linear_acceleration.y = ay
                msg.linear_acceleration.z = az
                msg.orientation_covariance = [0.05,0,0, 0,0.05,0, 0,0,0.05]
                msg.angular_velocity_covariance = [0.1,0,0, 0,0.1,0, 0,0,0.1]
                msg.linear_acceleration_covariance = [0.2,0.0,0.0, 0.0,0.2,0.0, 0.0,0.0,0.2]

                self.imu_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"IMU parse error: {e}")

def main(args=None):

    rclpy.init(args=args)

    node = ImuSerialNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            "Stopping IMU node"
        )

    finally:

        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()