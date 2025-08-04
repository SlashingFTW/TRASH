import rclpy
from rclpy.node import Node
import cv2
from ultralytics import YOLO
import os
import time
from ament_index_python.packages import get_package_share_directory
import os

class CVWebcamNode(Node):
    def __init__(self):
        super().__init__('cv_webcam_node')
        self.get_logger().info('YOLOv12 Webcam Node starting...')

        # Adjust path to your model if needed
        
        base_path = get_package_share_directory('trash_project')
        model_path = os.path.join(base_path, 'models', 'best_v7.pt')
        model = YOLO(model_path)

        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            self.get_logger().error("Failed to open webcam.")
            return

        start_time = time.time()
        while True:
            ret, frame = cap.read()
            if not ret:
                self.get_logger().error("Failed to read frame from webcam.")
                break

            results = model(frame)[0]
            annotated = results.plot()

            cv2.imshow("YOLOv12 Webcam", annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Exiting on user request.")
                break
            if time.time() - start_time > 10:
                self.get_logger().info("10 seconds elapsed. Exiting.")
                break

        cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info("Webcam session complete.")

def main(args=None):
    rclpy.init(args=args)
    node = CVWebcamNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
