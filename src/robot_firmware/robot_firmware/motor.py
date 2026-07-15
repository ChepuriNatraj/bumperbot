
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
        self.declare_parameter("max_linear", 0.5)      # m/s
        self.declare_parameter("max_angular", 0.5)     # rad/s
        self.declare_parameter("max_pwm", 110)
        self.declare_parameter("pwm_step", 10)          # max PWM change per cycle
        self.declare_parameter("cmd_timeout", 1.0)      # seconds - stop if no /cmd_vel
        self.declare_parameter("control_period", 0.02)  # 50 Hz
 
        self.max_linear = self.get_parameter("max_linear").value
        self.max_angular = self.get_parameter("max_angular").value
        self.max_pwm = self.get_parameter("max_pwm").value
        self.min_pwm = 30    # Adjust later (25–40)
        self.pwm_step = self.get_parameter("pwm_step").value
        self.cmd_timeout = self.get_parameter("cmd_timeout").value
        control_period = self.get_parameter("control_period").value
 
        # Smoothed PWM state
        self.left_pwm = 0
        self.right_pwm = 0
        self.kick_active = False
        self.kick_start_time = 0.0
        self.kick_duration = 0.08 
        self.kick_active_left = False
        self.kick_start_time_left = 0.0
        self.kick_active_right = False
        self.kick_start_time_right = 0.0
 
        # Latest velocity command (continuously updated by cmd_callback)
        self.linear = 0.0
        self.angular = 0.0
        self.last_cmd_time = time.time()
        self.last_print_time = time.time()
 
        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baud_rate").value
 
        self.ser = serial.Serial(port, baud, timeout=0.05)
        time.sleep(2)  # let the Arduino reset after opening the port
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
 
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
        self.last_cmd_time = time.time()

    def apply_min_pwm_kick(self, target, side):
        """Apply min_pwm only briefly when starting from rest, to break
        static friction. Once moving, let the true proportional (possibly
        slow) target through unmodified so slow commands stay slow."""
        current = self.left_pwm if side == 'left' else self.right_pwm
        now = time.time()

        if target == 0:
            if side == 'left':
                self.kick_active_left = False
            else:
                self.kick_active_right = False
            return 0

        starting_from_rest = abs(current) < 5

        if side == 'left':
            kick_active = self.kick_active_left
            kick_start = self.kick_start_time_left
        else:
            kick_active = self.kick_active_right
            kick_start = self.kick_start_time_right

        if starting_from_rest:
            if not kick_active:
                kick_active = True
                kick_start = now

            if now - kick_start < self.kick_duration:
                floor = self.min_pwm if target > 0 else -self.min_pwm
                result = floor if abs(target) < self.min_pwm else target
            else:
                kick_active = False
                result = target
        else:
            kick_active = False
            result = target

        if side == 'left':
            self.kick_active_left = kick_active
            self.kick_start_time_left = kick_start
        else:
            self.kick_active_right = kick_active
            self.kick_start_time_right = kick_start

        return result
 
    def update_motor(self):
        """Runs on a fixed timer regardless of /cmd_vel arrival rate.
        This is what makes behavior 'continuous': every tick it looks at
        the freshest linear/angular values, and independently enforces

        the watchdog stop if they've gone stale."""
        now = time.time()
        if hasattr(self, '_last_tick') and now - self._last_tick > 0.05:
            self.get_logger().warn(f"update_motor stall: {now - self._last_tick:.3f}s gap")
        self._last_tick = now

    
        # Watchdog: if no fresh command recently, force stop.
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            linear = 0.0
            angular = 0.0
        else:
            linear = self.linear
            angular = self.angular
 
        # Deadband
        if abs(linear) < 0.001:
            linear = 0.0
        if abs(angular) < 0.001:
            angular = 0.0
 
        # Velocity limits
        linear = max(-self.max_linear, min(self.max_linear, linear))
        angular = max(-self.max_angular, min(self.max_angular, angular))
 
        if linear == 0.0 and angular == 0.0:
            self.left_pwm = 0
            self.right_pwm = 0
            self._safe_write(b"0,0\n")
            if time.time() - self.last_print_time >= 10.0:
                self.get_logger().info("STOP (0,0)")
                self.last_print_time = time.time()
            return
 
        # Normalize to [-1, 1] then scale to PWM range
        linear_norm = linear / self.max_linear
        angular_norm = angular / self.max_angular
 
        target_left = int((linear_norm - angular_norm) * self.max_pwm)
        target_right = int((linear_norm + angular_norm) * self.max_pwm)

            # Apply minimum PWM only as a brief kick to break static friction,
            # not as a permanent floor — otherwise slow commands (e.g. during
            # collision_monitor "approach" mode or spin recovery) get forced
            # up to min_pwm every cycle and never actually crawl slowly.
        target_left = self.apply_min_pwm_kick(target_left, 'left')
        target_right = self.apply_min_pwm_kick(target_right, 'right')
    
        # Smooth ramp so PWM doesn't jump instantly
        self.left_pwm += max(
            -self.pwm_step, min(self.pwm_step, target_left - self.left_pwm)
        )
        self.right_pwm += max(
            -self.pwm_step, min(self.pwm_step, target_right - self.right_pwm)
        )
 
        cmd = f"{self.left_pwm},{self.right_pwm}\n"
        self._safe_write(cmd.encode())
        if time.time() - self.last_print_time >= 10.0:
            self.get_logger().info(
                f"Current cmd: lin={linear:.2f} ang={angular:.2f} -> "
                f"L={self.left_pwm} R={self.right_pwm}"
            )
            self.last_print_time = time.time()
 
    def _safe_write(self, data: bytes):
        """Serial writes can fail if the USB connection drops - don't let
        that crash the whole node, just log it and keep trying next tick."""
        try:
            self.ser.write(data)
            self.ser.flush()
        except serial.SerialException as e:
            self.get_logger().warn(f"Serial write failed: {e}")
 
    def read_encoder(self):
        try:
            while self.ser.in_waiting:
                line = self.ser.readline()
                if not line.endswith(b"\n"):
                    break  # incomplete line, wait for more data next cycle
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
 
    def destroy_node(self):
        try:
            if self.ser.is_open:
                self.ser.write(b"STOP\n")
                self.ser.flush()
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()
 
 
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
 