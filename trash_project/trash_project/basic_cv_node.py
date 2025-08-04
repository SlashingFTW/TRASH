import rclpy
from rclpy.node import Node
import cv2
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os
from std_msgs.msg import String

class CVNode(Node):
    def __init__(self):
        super().__init__('cv_node')
        self.get_logger().info('YOLO CV Node starting...')

        # Publisher
        self.publisher_ = self.create_publisher(String, 'cv_direction', 10)

        # Load model
        base_path = get_package_share_directory('trash_project')
        model_path = os.path.join(base_path, 'models', 'best_v7.pt')
        self.model = YOLO(model_path)

        # Open webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error(" Failed to open webcam.")
            return

        # Set frame size
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        # Check frame size
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.get_logger().info(f" Webcam opened at resolution {self.width}x{self.height}")

        # Start detection loop
        self.detect_loop()

    def detect_loop(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn(" Failed to read frame.")
                break

            line_thickness = 1
            line_color = (0,255,0) # green

            # Draw guide lines
            third_w = self.width // 3
            half_h = self.height // 2

            cv2.line(frame, (third_w, 0), (third_w, self.height), line_color, line_thickness)        # Left vertical            
            cv2.line(frame, (2 * third_w, 0), (2 * third_w, self.height), line_color, line_thickness)  # Right vertical            
            cv2.line(frame, (0, half_h), (self.width, half_h), line_color, line_thickness)            # Center horizontal

            # Run YOLO
            results = self.model(frame, conf=0.65, iou=0.8)[0]

            # Determine position of center of first object (if any)
            for box in results.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # Draw center point
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)  # red dot at center

                # Determine region
                region = "left" if cx < third_w else "right" if cx > 2 * third_w else "center"

                msg = String()
                msg.data = region
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published: {region}")
                break  # Only process the first detection


            # Show annotated frame
            cv2.imshow("YOLO Webcam Detection", results.plot())

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
