import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class amrcontroller(Node):
    def __init__(self):
        super().__init__('amr_control')

        self.serial_port_path = '/dev/ttyUSB1'  # แก้ตามเครื่องคุณ
        self.serial_baudrate = 115200
        self.serial_timeout = 1

        self.serial_port = None
        self.connect_serial()

        self.last_linear = None
        self.last_angular = None

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)

    def connect_serial(self):
        while self.serial_port is None:
            try:
                self.serial_port = serial.Serial(self.serial_port_path, self.serial_baudrate, timeout=self.serial_timeout)
                self.get_logger().info("Serial port connected.")
            except serial.SerialException:
                self.get_logger().warn("Serial port not available. Retrying in 2 seconds...")
                time.sleep(2)

    def listener_callback(self, msg):
        linear = round(msg.linear.x, 3)
        angular = round(msg.angular.z, 3)

        # ถ้าไม่มีการเปลี่ยนแปลง จะไม่ส่ง
        if linear == self.last_linear and angular == self.last_angular:
            return

        self.last_linear = linear
        self.last_angular = angular
        command = f"{linear:.3f},{angular:.3f}\n"

        # ส่งเฉพาะเมื่อ Serial ยังเชื่อมอยู่
        try:
            if self.serial_port and self.serial_port.is_open:
                self.get_logger().info(f"ส่งไปยัง Serial: {command.strip()}")
                self.serial_port.write(command.encode('utf-8'))
            else:
                self.get_logger().warn("Serial port closed. Trying to reconnect...")
                self.serial_port = None
                self.connect_serial()
        except serial.SerialException:
            self.get_logger().error("Serial exception occurred. Reconnecting...")
            self.serial_port = None
            self.connect_serial()

def main(args=None):
    rclpy.init(args=args)
    node = amrcontroller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_port and node.serial_port.is_open:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()