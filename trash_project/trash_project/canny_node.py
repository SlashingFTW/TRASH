import rclpy
import cv2
import os
import time

from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class CVNode(Node):
    def __init__(self):
        super().__init__('cv_node')
        self.get_logger().info('YOLO CV Node starting...')

        # Variables
        self.ready_to_detect = False    # Wait for motor_node to say "ready"
        self.last_zone = None  
        self.zone_start_time = None
        self.detection_delay = 3  # seconds

        # Publisher for object zone (far_left to far_right)
        self.zone_pub = self.create_publisher(String, 'cv_zone', 10)
        self.material_pub = self.create_publisher(String, 'cv_material', 10)  # New topic for material class
        self.ack_pub = self.create_publisher(String, 'ack', 10)

        # Subscribe to motor_node feedback ("done" signals)
        self.subscription = self.create_subscription(String, 'cv_feedback', self.feedback_callback, 10)

        # Load YOLO model from shared package directory
        base_path = get_package_share_directory('trash_project')
        model_path = os.path.join(base_path, 'models', 'best_v9.pt')
        self.model = YOLO(model_path)

        # Open webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open webcam.")
            return

        self.can_send = True  # Control flag to prevent repeated publishing

        # Try to set max
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        # Read actual resolution
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f"Webcam opened at resolution {self.width}x{self.height}")

        # Start continuous detection loop
        self.detect_loop()

    def feedback_callback(self, msg):
        if msg.data == "ready":
            self.get_logger().info("Received 'ready' from motor_node. Starting detection loop.")
            self.ready_to_detect = True
            self.can_send = True

            # send Ack
            ack = String()
            ack.data = 'ack'
            self.ack_pub.publish(ack)
            self.get_logger().info('Sent Ack')

    # Fallback detection using Canny edge detection
    def detect_edges_and_center(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest)  # Bounding box for drawing
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return (cx, cy), (x, y, w, h), edges  # include bounding box
        return None, None, edges

    def get_zone_name(self, cx):
        width_percent = cx / self.width # mid point of object realtive to width of screen
        if width_percent < 0.2:
            return "far_left"
        elif width_percent < 0.4:
            return "left"
        elif width_percent < 0.6:
            return "center"
        elif width_percent < 0.8:
            return "right"
        else:
            return "far_right"

    def draw_guidelines(self, frame):
        # Draw vertical lines to divide frame into 5 equal zones
        fifth_w = self.width // 5  # 1/5th line
        line_color = (0, 255, 0)
        line_thickness = 1

        for i in range(1, 5):  # Draw lines into 5ths
            x = i * fifth_w
            cv2.line(frame, (x, 0), (x, self.height), line_color, line_thickness)

        # Draw horizontal center line
        half_h = self.height // 2
        cv2.line(frame, (0, half_h), (self.width, half_h), line_color, line_thickness)

    def detect_loop(self):
        # Waiting for motor_node to be in ready position
        while not self.ready_to_detect and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Failed to read frame.")
                break

            # Run YOLO object detection
            results = self.model(frame, conf=0.8, iou=0.8)[0]
            detected = False  # Flag to track whether YOLO succeeded

            # Look for first high-confidence detection
            for box in results.boxes:
                conf = float(box.conf[0])
                if conf < 0.8:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]

                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

                zone = self.get_zone_name(cx)

                # Zone tracking logic
                if zone == self.last_zone:
                    if self.zone_start_time and (time.time() - self.zone_start_time >= self.detection_delay):
                        if self.can_send:
                            self.zone_pub.publish(String(data=zone))
                            self.material_pub.publish(String(data=class_name))
                            self.get_logger().info(f"Published zone={zone}, material={class_name}")
                            self.can_send = False
                            self.last_zone = None
                            self.zone_start_time = None
                            self.ready_to_detect = False # Reset Ready; wait for next ready
                else:
                    self.last_zone = zone
                    self.zone_start_time = time.time()

                detected = True
                break

            # If no valid YOLO detection, fallback
            if not detected:
                center, bbox, edge_img = self.detect_edges_and_center(frame)
                if center and bbox:
                    cx, cy = center
                    x, y, w, h = bbox

                    # Draw red bounding box and label
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.putText(frame, "unknown", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                    zone = self.get_zone_name(cx)

                    # Apply same zone-delay logic
                    if zone == self.last_zone:
                        if self.zone_start_time and (time.time() - self.zone_start_time >= self.detection_delay):
                            if self.can_send:
                                self.zone_pub.publish(String(data=zone))
                                self.material_pub.publish(String(data="unknown"))
                                self.get_logger().info(f"Published fallback zone={zone}, material=unknown")
                                self.can_send = False
                                self.ready_to_detect = False 
                    else:
                        self.last_zone = zone
                        self.zone_start_time = time.time()

                cv2.imshow("Canny Edges", edge_img)

            self.draw_guidelines(frame) # Draw Guidelines
            cv2.imshow("YOLOv12 Webcam Detection", results.plot())  # Show annotated YOLO/camera frame

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("'q' pressed. Exiting...")
                break

        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = CVNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
