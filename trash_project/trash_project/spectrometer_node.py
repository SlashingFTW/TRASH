import rclpy
from rclpy.node import Node

import time
import os
import glob
import struct
import threading
import numpy as np
import serial
from scipy.spatial.distance import euclidean
from std_msgs.msg import String

SAMPLES = 3694  # Expected number of spectral data points
LIB_FOLDER = '~/ros2_ws/src/trash_project/trash_project/spectrometer_library'  # Path to library

class SpectrometerNode(Node):
    def __init__(self):
        super().__init__('spectrometer_node')
        self.get_logger().info("Spectrometer Node initializing...")

        # Publisher 
        self.result_pub = self.create_publisher(String, 'spectrometer_result', 10)

        self.last_material = None
        self.material_start_time = None
        self.publish_threshold = 3.0  # seconds
        self.ser = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()  # Ensures thread-safe access to latest_frame
        self.library = self.build_library(LIB_FOLDER)  # Load reference spectral library

        port = "/dev/ttyACM0"

        try:
            self.ser = serial.Serial(port, 115200, timeout=1)
            self.get_logger().info(f"Connected to {port}")

            self.set_integration_time(500)

            # Start background thread to read spectrometer data
            threading.Thread(target=self.serial_reader, daemon=True).start()

            # Timer to periodically process new frames
            self.timer = self.create_timer(0.5, self.process_frame)

        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")

    
    def set_integration_time(self, ms):
        """
        Sends a command to set the spectrometer's integration time.
        Adjust the command format as needed for your device.
        """
        try:
            command = f"ITIME {ms}\n"
            self.ser.write(command.encode('utf-8'))
            self.get_logger().info(f"Integration time set to {ms} ms")
        except Exception as e:
            self.get_logger().error(f"Failed to set integration time: {e}")

    def sync_and_read(self):
        """
        Waits for header bytes (0xAA 0x55) and reads one full frame of data.
        """
        while True:
            if self.ser.read(1) == b'\xAA' and self.ser.read(1) == b'\x55':
                data = self.ser.read(SAMPLES * 2)
                if len(data) == SAMPLES * 2:
                    return data

    def serial_reader(self):
        """
        Continuously reads frames from the serial port in a separate thread.
        """
        while True:
            frame = self.sync_and_read()
            with self.frame_lock:
                self.latest_frame = frame

    def build_library(self, folder):
        """
        Loads and normalizes all CSV files in the reference library folder.
        Returns a dict: {name: 2D array [wavelength, normalized_intensity]}.
        """
        paths = glob.glob(os.path.join(folder, "*.csv"))
        lib = {}
        for p in paths:
            name = os.path.splitext(os.path.basename(p))[0].replace(' ', '_')
            data = np.loadtxt(p, delimiter=',', skiprows=1)
            wl = data[:, 0]
            inten = data[:, 1]
            norm = (inten - inten.min()) / (inten.max() - inten.min())
            lib[name] = np.column_stack((wl, norm))
        return lib

    def process_frame(self):
        with self.frame_lock:
            frame = self.latest_frame
            self.latest_frame = None

        if not frame:
            return

        # Convert binary data to intensity values
        vals = struct.unpack('<' + 'H' * SAMPLES, frame)
        y = np.array(vals, dtype=float)

        y_disp = y.max() - y
        x = np.arange(len(y_disp))
        norm = (y_disp - y_disp.min()) / (y_disp.max() - y_disp.min())

        # Match against reference library
        best_key = None
        best_dist = float('inf')
        for key, arr in self.library.items():
            interp_ref = np.interp(x, arr[:, 0], arr[:, 1])
            dist = euclidean(norm, interp_ref)
            if dist < best_dist:
                best_key = key
                best_dist = dist

        # Classify material
        material_map = {
            "coke_can": "Aluminum",
            "RedBull_can": "Aluminum",
            "sprite_can": "Aluminum",
            "Brown_paper": "Cardboard",
            "cardboard": "Cardboard",
            "Glass_bottle": "Glass",
            "plastic": "Plastic",
            "noise": "Noise"
        }

        matched_category = "Unknown"
        if best_key:
            for keyword, material in material_map.items():
                if keyword.lower() in best_key.lower():
                    matched_category = material
                    break

        self.get_logger().info(f"Material: {matched_category} — Match: {best_key}, Distance: {best_dist:.3f}")

        # Ignore noise and reset timer
        if matched_category == "Noise":
            self.last_material = None
            self.material_start_time = None
            return

        # Start or continue timer for stable match
        current_time = time.time()
        if matched_category != self.last_material:
            self.last_material = matched_category
            self.material_start_time = current_time
            return

        if self.material_start_time and (current_time - self.material_start_time >= self.publish_threshold):
            msg = String()
            msg.data = matched_category
            self.result_pub.publish(msg)
            self.get_logger().info(f"[PUBLISHED] Spectrometer result: {matched_category}")
            self.last_material = None
            self.material_start_time = None

def main(args=None):
    rclpy.init(args=args)
    node = SpectrometerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
