# Import ROS 2 Python client libraries
import rclpy
from rclpy.node import Node

# Import standard ROS 2 message type for strings
from std_msgs.msg import String

# Import serial for UART communication and time for delays
import serial
import time

from enum import Enum

# CV sends 'left, right or senter' - FSM starts:
# - MOVE_TO_OBJECT     (go to location)
# - GRAB_OBJECT        (lower, grip)
# - LIFT_OBJECT        (raise)
# - MOVE_TO_SPECTROMETER
# - ANALYZE_OBJECT     (TODO: real service)
# - MOVE_TO_BIN        (metal or trash)
# - DROP_OBJECT        (lower, open grip)
# - RETURN_HOME        (reset pose)
# - RESET - IDLE

# Define a custom class for the motor controller node, inheriting from Node
class MotorNode(Node):
    def __init__(self):
        # Initialize the base Node class with the name 'motor_node'
        super().__init__('motor_node')

        self.state = ArmState.IDLE
        self.cv_target = None
        #self.spectrometer_result = None  
        self.last_command_sent = None
        self.fsm_timer = self.create_timer(1.0, self.run_state_machine)  # Runs every second
        self.create_timer(0.5, self.send_ready_feedback) # runs every half second while we wait for 'ack'

        # Initialize default internal states
        self.latest_command = "center"          # Stores latest direction from CV
        self.angles = [0, 0, 0, 180, 0, 0]        # Initial joint angles for t1 to t6 (home position)
        self.grip_open = True                   # Track whether the gripper is open
        self.waiting_response = False           # Flag to track if we are waiting on the serial response
        self.ready_position = "arm 0 20 10 90 82 0"
        self.skip_next_timer = False            # Tells timer to skip next command send
        self.state = ArmState.HOMING
        self.material_detected = None


        self.cv_ack_received = False
        self.last_command_failed = False

        # TODO: Movement commands to be set
        self.named_commands = {
            "home": "home",                     
            "ready_pos": "arm 0 20 10 90 82 0",
            # Zones
            "far_left": "far_left",
            "left": "left",        
            "center": "center",                     
            "right": "right",       
            "far_right": "far_right",   
            # Clearance
            "lift": "arm 0 -30 27 180 0 90",
            "recycable_clear": "",                  #TODO
            "trash_clear": "",                      #TODO
            # Locations
            "spectrometer": "arm 89 -65 50 160 30 90",
            "metal_bin": "bin:metal",                #TODO: Might have to change to arm depending on if bin is on >            
            "cardboard_bin": "bin:cardboard",                #TODO
            "plastic_bin": "bin:plastic",
            # End-Effector commands
            "ee_open": "set t6 0",
            "ee_close": "set t6 90"
        }

        # ==== ROS2 Setup =====
        self.get_logger().info("Motor Node started. Waiting for cv_direction messages...")
        # Publisher 
        self.cv_feedback_pub = self.create_publisher(String, 'cv_feedback', 10)
        self.state_pub = self.create_publisher(String, 'motor_state', 10)
        self.joint_pub = self.create_publisher(String, 'joint_angles', 10)
        self.serial_output_pub = self.create_publisher(String, 'serial_output', 10)

        # Subscriber 
        self.subscription = self.create_subscription(String,'cv_zone',self.direction_callback, 10)
        self.cv_ack_sub = self.create_subscription(String, 'ack', self.cv_ack_callback, 10)
        self.serial_cmd_sub = self.create_subscription(String, 'serial_command', self.serial_command_callback, 10)
        self.toggle_sub = self.create_subscription(String, 'gui_toggle', self.toggle_callback, 10)
        #self.create_subscription(String, 'spectrometer_result', self.spectro_callback, 10)
        self.create_subscription(String, 'cv_material', self.material_callback, 10)


        # Try to open the serial port connection
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            time.sleep(2)  # Give the serial connection time to initialize
            self.get_logger().info("Serial connection established.")
        except serial.SerialException as e:
            # If opening the serial port fails, log the error
            self.get_logger().error(f"Serial error: {e}")
            self.ser = None  # Avoid crashes later by checking this

        # Send a one-time arm command at startup (set position for all joints)
        self.send_named_command("home")  # Start by homing the arm

    # ----------------------------------- CV ---------------------------------------
    def cv_ack_callback(self, msg):
        self.get_logger().info(f"Recvied on cv_ack: {msg.data}")
        if msg.data == 'ack':
            self.get_logger().info('Ack Received')
            self.cv_ack_received = True

    def send_ready_feedback(self):
        if self.state == ArmState.IDLE and not self.cv_ack_received:
            msg = String()
            msg.data = "ready"
            self.cv_feedback_pub.publish(msg)

    def direction_callback(self, msg):
        command = msg.data.strip()
        self.get_logger().info(f"Direction received: {command}")

        valid_zones = ["far_left", "left", "center", "right", "far_right"]
        if self.state == ArmState.IDLE and command in valid_zones:
            self.cv_target = command
            self.state = ArmState.MOVE_TO_OBJECT

    def material_callback(self, msg):
        self.get_logger().info(f"[CV Material] Received: {msg.data}")
        self.material_detected = msg.data.strip()

    # --------------------------------- Motor Controller --------------------------
    def send_named_command(self, name):
        if name not in self.named_commands:
            self.get_logger().warn(f"Unknown command received: '{name}'")
            return

        # Only send if different from last OR last attempt failed/timed out
        if self.last_command_sent == name and not self.last_command_failed:
            self.get_logger().info(f"Skipping redundant command '{name}' (already sent)")
            return

        cmd = self.named_commands[name] + "\n"
        self.ser.write(cmd.encode())
        self.get_logger().info(f"Sent named command '{name}': {cmd.strip()}")
        self.last_command_sent = name

        self.waiting_response = True
        self.last_command_failed = not self.wait_for_response()
        self.skip_next_timer = True

        if name == 'home':
            msg = String()
            msg.data = 'home'
            self.joint_pub.publish(msg)


    # Wait and log responses from the serial device (up to 20 seconds, Finished, Failed)
    def wait_for_response(self):
        start_time = time.time()
        finished = False
        #self.waiting_response = False


        while time.time() - start_time < 20:  # 20-second timeout
            if self.ser.in_waiting:
                # Read and decode the line from serial
                response = self.ser.readline().decode(errors="ignore").strip()

                # Log all serial output
                if response:
                    self.get_logger().info(f"[SERIAL] {response}")
                    msg = String()
                    msg.data = response
                    self.serial_output_pub.publish(msg)

                # Check if response indicates command completion
                if "Finished" in response:
                    finished = True
                    break
            else:
                time.sleep(0.05)  # Light wait to avoid busy loop

        if not finished:
            self.get_logger().warn("No 'Finished' response or timed out.")
            self.waiting_response = False
            return False

        self.waiting_response = False
        return True

    def serial_command_callback(self, msg):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((msg.data + "\n").encode())
                self.get_logger().info(f"Sent to UART: {msg.data}")
            except Exception as e:
                self.get_logger().error(f"UART send error: {e}")

    def toggle_callback(self, msg):
        if msg.data == 'off':
            self.get_logger().info("System toggled OFF")
        elif msg.data == 'on':
            self.get_logger().info("System toggled ON")

    def run_state_machine(self):
        if self.waiting_response or self.ser is None:
            return  # Wait for current serial action to complete

        if self.state == ArmState.HOMING:
            self.send_named_command("home")
            self.state = ArmState.GO_TO_READY
            self.get_logger().info("GO_TO_READY")
            msg = String()
            msg.data = self.state.name
            self.state_pub.publish(msg)
            return

        elif self.state == ArmState.GO_TO_READY:
            self.send_named_command("ready_pos")  # This is your ready position
            self.state = ArmState.IDLE
            self.get_logger().info("IDLE (Waiting for cv_node)")
            self.send_ready_feedback()      # Send cv_node, "ready"
            return

        elif self.state == ArmState.IDLE:
            return

        elif self.state == ArmState.MOVE_TO_OBJECT:
            self.send_named_command(self.cv_target)     # Go to far_left/left/far_right/right/center
            self.state = ArmState.GRAB_OBJECT
            self.get_logger().info("GRAB_OBJECT")

        elif self.state == ArmState.GRAB_OBJECT:
            self.send_named_command("ee_close")         # Close gripper
            self.state = ArmState.LIFT_OBJECT
            self.get_logger().info("LIFT_OBJECT")

        elif self.state == ArmState.LIFT_OBJECT:
            self.send_named_command("lift")
            self.state = ArmState.MOVE_TO_SPECTROMETER
            self.get_logger().info("MOVE_TO_SPECTROMETER")

        elif self.state == ArmState.MOVE_TO_SPECTROMETER:
            self.send_named_command("spectrometer")
            self.state = ArmState.ANALYZE_OBJECT
            self.get_logger().info("ANALYZE_OBJECT")

        elif self.state == ArmState.ANALYZE_OBJECT:
            self.get_logger().info("Waiting for CV material classification...")

            start_time = time.time()
            timeout = 30.0

            while rclpy.ok() and self.material_detected is None and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)

            if self.material_detected is None:
                self.get_logger().warn("CV material did not return in time. Defaulting to trash.")
                self.material_detected = "Unknown"

            self.waiting_response = False
            self.state = ArmState.MOVE_TO_BIN
            self.get_logger().info("MOVE_TO_BIN")
            self.get_logger().info(f"Material: {self.material_detected}")

        elif self.state == ArmState.MOVE_TO_BIN:
            self.get_logger().info("In MOVE_TO_BIN")
            mat = self.material_detected.strip().lower()
            if mat == "metal" or mat == "aluminum":
                self.get_logger().info("Going to metal bin")
                bin_cmd = "metal_bin"
            elif mat == "cardboard":
                self.get_logger().info("Going to cardboard bin")
                bin_cmd = "cardboard_bin"
            else:
                self.get_logger().info("Defaulting to trash bin")
                bin_cmd = "plastic_bin"  

            self.send_named_command(bin_cmd)
            self.state = ArmState.DROP_OBJECT
            self.get_logger().info("DROP_OBJECT")


        elif self.state == ArmState.DROP_OBJECT:
            self.send_named_command("ee_open")
            self.grip_open = True
            self.state = ArmState.RETURN_HOME
            self.get_logger().info("RETURN_HOME")

        elif self.state == ArmState.RETURN_HOME:
            self.ser.write(b"set t3 30\n")
            time.sleep(0.5)
            self.state = ArmState.RESET
            self.get_logger().info("RESET")

        elif self.state == ArmState.RESET:
            self.last_command_sent = None
            self.cv_target = None
            self.spectrometer_result = None
            self.cv_ack_received = False 
            self.state = ArmState.HOMING
            self.get_logger().info("Go Home")

class ArmState(Enum):
    HOMING = -1
    IDLE = 0
    MOVE_TO_OBJECT = 1
    GRAB_OBJECT = 2
    LIFT_OBJECT = 3
    MOVE_TO_SPECTROMETER = 4
    ANALYZE_OBJECT = 5
    MOVE_TO_BIN = 6
    DROP_OBJECT = 7
    RETURN_HOME = 8
    RESET = 9
    GO_TO_READY = 10

def main(args=None):
    rclpy.init(args=args)         # Initialize ROS 2 Python system
    node = MotorNode()            # Create an instance of our motor node
    try:
        rclpy.spin(node)          # Keep the node running and responsive
    except KeyboardInterrupt:
        pass                      # Handle Ctrl+C gracefully
    node.destroy_node()           # Clean up resources (e.g., serial)
    rclpy.shutdown()              # Shutdown ROS 2

if __name__ == '__main__':
    main()
