import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import time
import threading

class BasicMotorNode(Node):
    def __init__(self):
        super().__init__('basic_motor_node')
        self.get_logger().info("Basic Motor Node started.")

        try:
            self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
            time.sleep(2)
            self.get_logger().info("Serial connected on /dev/ttyUSB0")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            self.ser = None
            return

        self.input_thread = threading.Thread(target=self.console_loop, daemon=True)
        self.input_thread.start()

    def console_loop(self):
        while rclpy.ok():
            try:
                command = input("Enter UART command: ").strip()
                if not command or self.ser is None:
                    continue

                self.ser.write((command + '\n').encode())
                self.get_logger().info(f"Sent: {command}")

                # Read output for up to 5 seconds or until 'Finished'
                start_time = time.time()
                finished = False

                print("Waiting for response...")
                while time.time() - start_time < 5.0:
                    if self.ser.in_waiting:
                        line = self.ser.readline().decode(errors='ignore').strip()
                        if line:
                            print(f"SERIAL: {line}")
                            if 'Finished' in line:
                                finished = True
                                break
                    else:
                        time.sleep(0.05)

                if finished:
                    print("Finished response received.\n")
                else:
                    print("Timeout after 5 seconds.\n")

            except Exception as e:
                self.get_logger().error(f"Console error: {e}")
                break

def main(args=None):
    rclpy.init(args=args)
    node = BasicMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
