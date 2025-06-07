#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess
import signal

class EmergencyStopWatcher(Node):
    def __init__(self):
        super().__init__('emergency_stop_watcher')

        self.process = None
        self.system_running = False

        self.sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_callback,
            10
        )

        self.start_system()

    def start_system(self):
        if self.process is None or self.process.poll() is not None:
            self.process = subprocess.Popen(
                ["ros2", "launch", "cr_bringup", "system_bringup.launch.py"]
            )
            self.system_running = True
            self.get_logger().info("✅ Start system.")

    def stop_system(self):
        if self.process is not None and self.process.poll() is None:
            self.get_logger().warn("🛑 Arrested system wait...")
            self.process.send_signal(signal.SIGINT)
            self.process.wait()
            self.get_logger().info("☠️ System Killed.")
        self.system_running = False

    def emergency_callback(self, msg):
        if msg.data:  # EMERGENCY STOP
            if self.system_running:
                self.stop_system()
            else:
                self.get_logger().warn("recived false but the system is already stopped.")
        else:  # RIPARTI
            if not self.system_running:
                self.get_logger().info("🔁 Restart again LET'S GO.")
                self.start_system()
            else:
                self.get_logger().info("Restart ignored, the system is already activate.")

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopWatcher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
