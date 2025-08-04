import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt6.QtWidgets import (QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout, QLineEdit, QTextEdit)
from PyQt6.QtCore import QTimer

class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')

        self.cv_zone = "N/A"
        self.spectro_result = "N/A"
        self.motor_state = "N/A"
        self.computer_vision_material = "N/A"
        self.toggle_state = False
        self.serial_output = ""
        self.joint_angles = {
            't1': 'N/A', 't2': 'N/A', 't3': 'N/A',
            't4': 'N/A', 't5': 'N/A', 't6': 'N/A'
        }

        self.create_subscription(String, 'cv_zone', self.cv_callback, 10)
        self.create_subscription(String, 'spectrometer_result', self.spectro_callback, 10)
        self.create_subscription(String, 'motor_state', self.motor_callback, 10)
        self.create_subscription(String, 'joint_angles', self.joint_callback, 10)
        self.create_subscription(String, 'serial_output', self.serial_output_callback, 10)
        self.create_subscription(String, 'cv_material', self.material_callback, 10)


        self.toggle_pub = self.create_publisher(String, 'gui_toggle', 10)
        self.serial_pub = self.create_publisher(String, 'serial_command', 10)

    def cv_callback(self, msg):
        self.cv_zone = msg.data

    def spectro_callback(self, msg):
        self.spectro_result = msg.data

    def motor_callback(self, msg):
        self.motor_state = msg.data

    def joint_callback(self, msg):
        if msg.data == 'home':
            self.joint_angles = {
                't1': 0, 't2': 0, 't3': 0,
                't4': 180, 't5': 0, 't6': 0
            }
        else:
            try:
                updates = msg.data.strip().split()
                for joint in updates:
                    name, val = joint.split(":")
                    self.joint_angles[name.strip()] = int(val.strip())
            except Exception as e:
                self.get_logger().warn(f"Failed to parse joint_angles: {msg.data}")

    def serial_output_callback(self, msg):
        self.serial_output = msg.data
    
    def material_callback(self, msg):
        self.computer_vision_material = msg.data
        self.get_logger().info(f"CV Material: {self.computer_vision_material}")

    def publish_toggle(self):
        msg = String()
        msg.data = 'on' if self.toggle_state else 'off'
        self.toggle_pub.publish(msg)
        self.get_logger().info(f"[GUI] Published toggle: {msg.data}")

    def send_serial_command(self, command: str):
        msg = String()
        msg.data = command
        self.serial_pub.publish(msg)
        self.get_logger().info(f"[GUI] Sent: {command}")

class SimpleGUI(QWidget):
    def __init__(self, gui_node):
        super().__init__()
        self.gui_node = gui_node
        self.setWindowTitle("TrashBot Control Panel")

        # Left side status labels
        self.cv_label = QLabel("CV Zone: N/A")
        self.material_label = QLabel("Material: N/A")
        self.spectro_label = QLabel("Spectrometer: N/A")
        self.motor_label = QLabel("Motor State: N/A")

        left_layout = QVBoxLayout()
        left_layout.addWidget(self.cv_label)
        left_layout.addWidget(self.material_label) 
        left_layout.addWidget(self.spectro_label)
        left_layout.addWidget(self.motor_label)
        left_layout.addStretch()

        # Right side joint angle labels
        self.joint_labels = {}
        right_layout = QVBoxLayout()
        for joint in ['t1','t2','t3','t4','t5','t6']:
            lbl = QLabel(f"{joint.upper()}: N/A")
            self.joint_labels[joint] = lbl
            right_layout.addWidget(lbl)
        right_layout.addStretch()

        # Combine left and right side
        row_layout = QHBoxLayout()
        row_layout.addLayout(left_layout)
        row_layout.addLayout(right_layout)

        # Manual control input fields
        self.motor_inputs = {}
        manual_layout = QGridLayout()
        manual_layout.addWidget(QLabel("Manual Control (Set Angles)"), 0, 0, 1, 2)

        for i, joint in enumerate(['t1','t2','t3','t4','t5','t6']):
            lbl = QLabel(f"{joint.upper()}:")
            entry = QLineEdit()
            entry.setPlaceholderText("Hello World!")
            self.motor_inputs[joint] = entry
            manual_layout.addWidget(lbl, i+1, 0)
            manual_layout.addWidget(entry, i+1, 1)

        self.send_btn = QPushButton("Send Angles")
        self.send_btn.clicked.connect(self.send_motor_commands)
        manual_layout.addWidget(self.send_btn, 7, 0, 1, 2)

        # Serial output display
        self.serial_output_display = QTextEdit()
        self.serial_output_display.setReadOnly(True)
        self.serial_output_display.setPlaceholderText("Serial output from ESP32 will appear here...")

        # ON/OFF toggle button
        self.toggle_btn = QPushButton("Turn ON")
        self.toggle_btn.setCheckable(True)
        self.toggle_btn.clicked.connect(self.toggle_pressed)

        # Main layout assembly
        main_layout = QVBoxLayout()
        main_layout.addLayout(row_layout)
        main_layout.addLayout(manual_layout)
        main_layout.addWidget(self.serial_output_display)
        main_layout.addWidget(self.toggle_btn)
        self.setLayout(main_layout)

        # GUI update loop
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_labels)
        self.timer.start(500)

    def send_motor_commands(self):
        for joint, line_edit in self.motor_inputs.items():
            angle_str = line_edit.text().strip()
            if angle_str:
                try:
                    angle = int(angle_str)
                    command = f"set {joint} {angle}"
                    self.gui_node.send_serial_command(command)
                    line_edit.clear()
                except ValueError:
                    print(f"Invalid input for {joint}: {angle_str}")

    def toggle_pressed(self):
        self.gui_node.toggle_state = self.toggle_btn.isChecked()
        self.toggle_btn.setText("Turn OFF" if self.toggle_btn.isChecked() else "Turn ON")
        self.gui_node.publish_toggle()

    def update_labels(self):
        self.cv_label.setText(f"CV Zone: {self.gui_node.cv_zone}")
        self.material_label.setText(f"CV Material: {self.gui_node.computer_vision_material}")
        self.spectro_label.setText(f"Spectrometer Material: {self.gui_node.spectro_result}")
        self.motor_label.setText(f"Motor State: {self.gui_node.motor_state}")
        for joint, lbl in self.joint_labels.items():
            lbl.setText(f"{joint.upper()}: {self.gui_node.joint_angles.get(joint, 'N/A')}")

        # Append new serial output if it's different from the last
        existing = self.serial_output_display.toPlainText().splitlines()
        new_line = self.gui_node.serial_output
        if not existing or (existing and new_line != existing[-1]):
            self.serial_output_display.append(new_line)

def main(args=None):
    rclpy.init(args=args)
    gui_node = GuiNode()
    app = QApplication(sys.argv)
    gui = SimpleGUI(gui_node)
    gui.show()

    # Run ROS in background
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(gui_node, timeout_sec=0))
    timer.start(10)

    app.exec()
    gui_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
