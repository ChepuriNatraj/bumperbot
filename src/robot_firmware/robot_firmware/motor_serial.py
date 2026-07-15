#!/usr/bin/env python3
 
import serial
import time
 
import rclpy
from rclpy.node import Node
 
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64MultiArray
 
 
class MotorSerial(Node):
 
    def __init__(self):
        super().__init__("motor_serial")
        
 
        # ---- Parameters (tunable at launch, no code edits needed) ----
        self.declare_parameter("serial_port", "/dev/ttyACM1")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("max_linear", 0.4)      # m/s
        self.declare_parameter("max_angular", 0.4)     # rad/s
          # max PWM change per cycle
        self.declare_parameter("cmd_timeout", 0.3)      # seconds - stop if no /cmd_vel
        self.declare_parameter("control_period", 0.01) #100 Hz
        
        self.declare_parameter("wheel_base", 0.535)
 
        self.max_linear = self.get_parameter("max_linear").value
        self.max_angular = self.get_parameter("max_angular").value
   
        
        self.wheel_base = self.get_parameter("wheel_base").value
        
        self.cmd_timeout = self.get_parameter("cmd_timeout").value
        control_period = self.get_parameter("control_period").value
 
      
 
        # Latest velocity command (continuously updated by cmd_callback)
        self.linear = 0.0
        self.angular = 0.0
        self.linear_f = 0.0
        self.angular_f = 0.0
        self.last_cmd_time = time.monotonic()
        self.last_print_time = time.monotonic()
 
        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baud_rate").value
 
        self.ser = serial.Serial(port, baud, timeout=0.05)
        time.sleep(2)  # let the Arduino reset after opening the port
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self._serial_buf = b""
 
        self.enc_pub = self.create_publisher(Int64MultiArray, "/encoder_ticks", 10)
 
        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_callback, 10
        )
 
        # Two independent timers:
        # - update_motor: always runs, always reflects the *latest* cmd_vel,
        #   and zeroes out on its own if nothing new has arrived (watchdog).
        # - read_encoder: drains whatever the Arduino has sent back.
        self.motor_timer = self.create_timer(control_period, self.update_motor)
        self.timer = self.create_timer(control_period, self.read_encoder)
 
        self.get_logger().info(
            f"MotorSerial started on {port} @ {baud} baud, "
            f"cmd_timeout={self.cmd_timeout}s"
        )
 
    def cmd_callback(self, msg: Twist):
        """Called every time a /cmd_vel message arrives - just records the
        latest target. Does NOT drive the motors directly, so a single
        message never gets 'stuck' being replayed forever, and a stream
        of messages is naturally followed continuously."""
        self.linear = msg.linear.x
        self.angular = msg.angular.z
        self.last_cmd_time = time.monotonic()
     
    def update_motor(self):
        """Runs on a fixed timer regardless of /cmd_vel arrival rate.
        This is what makes behavior 'continuous': every tick it looks at
        the freshest linear/angular values, and independently enforces
        the watchdog stop if they've gone stale."""
 
        # Watchdog: if no fresh command recently, force stop.
        if time.monotonic() - self.last_cmd_time > self.cmd_timeout:
            linear = 0.0
            angular = 0.0
        else:
            linear = self.linear
            angular = self.angular
 
        # Deadband
        if abs(linear) < 0.005:
            linear = 0.0
        if abs(angular) < 0.005:
            angular = 0.0
 
        # Velocity limits
        linear = max(-self.max_linear, min(self.max_linear, linear))
        angular = max(-self.max_angular, min(self.max_angular, angular))

        # Low-pass filter
        alpha = 0.2

       

        self.linear_f += alpha * (linear - self.linear_f)
        self.angular_f += alpha * (angular - self.angular_f)

        linear = self.linear_f
        angular = self.angular_f

        # Stop check
        if abs(linear) < 0.005 and abs(angular) < 0.005:
            
            self._safe_write(b"0.0,0.0\n")
            if time.monotonic() - self.last_print_time >= 10.0:
                self.get_logger().info("STOP (0,0)")
                self.last_print_time = time.monotonic()
            return
 
        

        left_vel = linear - angular * self.wheel_base / 2.0
        right_vel = linear + angular * self.wheel_base / 2.0

        max_wheel_speed = self.max_linear

        left_vel = max(-max_wheel_speed, min(max_wheel_speed, left_vel))
        right_vel = max(-max_wheel_speed, min(max_wheel_speed, right_vel))

        cmd = f"{left_vel:.4f},{right_vel:.4f}\n"

        self._safe_write(cmd.encode())
        if time.monotonic() - self.last_print_time > 2.0:
            self.get_logger().info(
                f"Target L={left_vel:.3f}  Target R={right_vel:.3f}"
            )
            self.last_print_time = time.monotonic()

        # Scale to [min_pwm, max_pwm] while preserving the positive/negative direction

        # Smooth ramp so PWM doesn't jump instantly
        
        
        
 
    def _safe_write(self, data: bytes):
        """Serial writes can fail if the USB connection drops - don't let
        that crash the whole node, just log it and keep trying next tick."""
        try:
            
            self.ser.write(data)
            
        except serial.SerialException as e:
            self.get_logger().warn(f"Serial write failed: {e}")
 
    def read_encoder(self):
        try:
            n = self.ser.in_waiting
            if n:
                self._serial_buf += self.ser.read(n)  # never blocks, reads only what's there

            while b"\n" in self._serial_buf:
                line, self._serial_buf = self._serial_buf.split(b"\n", 1)
                line = line.decode("utf-8", errors="ignore").strip()

                if not line.startswith("E,"):
                    continue

                data = line.split(",")
                if len(data) != 3:
                    continue

                left = int(data[1])
                right = int(data[2])

                msg = Int64MultiArray()
                msg.data = [left, right]
                self.enc_pub.publish(msg)

        except (serial.SerialException, ValueError) as e:
            self.get_logger().warn(f"Encoder read failed: {e}", throttle_duration_sec=5)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = MotorSerial()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()