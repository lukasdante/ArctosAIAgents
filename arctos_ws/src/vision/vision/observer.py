from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import ultralytics.engine.results

import rclpy
from arctos_ws.src.utils.utils.base import BaseNode
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from rcl_interfaces.msg import ParameterDescriptor

class Observer(BaseNode):
    def __init__(self):
        super().__init__("observer")

        try:
            # Declare parameters for the node
            self.declare_parameter('model', 'yolo11s.pt', ParameterDescriptor(description='Object detection pretrained model can be .engine, .pt, or .onnx.'))
            self.declare_parameter('fps', 15, ParameterDescriptor(description='Frames per second (fps) of the camera feed.'))
            self.declare_parameter('video', True, ParameterDescriptor(description='If True, the observer node continuously detects objects in a specified fps.'))
            self.declare_parameter('threshold', 60, ParameterDescriptor(description='The object detection threshold of the model.'))

            # Set node parameters
            self.model = YOLO(f"models/{self.get_parameter('model').get_parameter_value().string_value}")  # Pre-trained YOLOv8 Nano weights
            self.frequency = round(1 / self.get_parameter('fps').get_parameter_value().integer_value, 2)
            self.video = self.get_parameter('video').get_parameter_value().bool_value
            self.threshold = self.get_parameter('threshold').get_parameter_value().integer_value
            # Initialize camera
            self.cap = cv2.VideoCapture(0)
            # try:
            #     self.cap = cv2.VideoCapture(0)
            # except:
            #     pass

            if not self.cap.isOpened():
                self.get_logger().error("Could not access the default camera.")

            # ROS2 publisher for annotated images
            self.image_pub = self.create_publisher(Image, "observation/image", 10)
            self.class_pub = self.create_publisher(String, "observation/classes", 10)
            self.oneshot_pub = self.create_publisher(String, "observation/oneshot_response", 10)
            self.parameter_subscriber = self.create_subscription(String, 'conversation/parameters', self.respond_parameter, 10)


            # OpenCV bridge to convert between OpenCV images and ROS2 images
            self.bridge = CvBridge()

            # Create a timer to process the camera feed if video is set to True
            if self.video:
                self.timer = self.create_timer(self.frequency, self.process_frame)

            self.get_logger().info(f'Observer initialized.')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize observer node: {e}')

    def respond_parameter(self, msg):
        try:
            params = self.parse_params(msg)
            # If not changing parameters return immediately
            if 'change_parameter' not in params.keys():
                raise Exception()
            
            self.change_parameter(params)
            if self.get_parameter('model').get_parameter_value().string_value == 'YOLOv11':
                model_name = 'yolo11s.pt'
            if self.get_parameter('model').get_parameter_value().string_value == 'YOLOv8':
                model_name = 'yolov8n.pt'

            self.model = YOLO(f"models/{model_name}")  # Pre-trained YOLOv8 Nano weights
            self.frequency = round(1 / self.get_parameter('fps').get_parameter_value().integer_value, 2)
        except:
            pass

        try:
            if 'current look' in params.values():
                self.process_frame(oneshot=True)

        except:
            pass

    def process_frame(self, oneshot=False):
        """ Obtains the predictions from model. """
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame.")
            return

        # Perform inference using YOLOv8
        try:
            results = self.model(frame)
            result: ultralytics.engine.results.Results = results[0]
            annotated_frame = result.plot()

            self.obtain_classes(result, oneshot)

            # Publish the annotated frame
            image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.image_pub.publish(image_msg)
        
        except Exception as e:
            self.get_logger().error(f'Failed to process frame: {e}')

    def obtain_classes(self, result: ultralytics.engine.results.Results, oneshot=False):
        """ Obtain a list of classes detected by the model. """
        classes = result.boxes.cls
        names: dict = result.names

        # Obtain classes
        detected_classes = [names.get(key.item()) for key in classes]
        
        # Create a single string for the detected classes
        detected_classes = ','.join(detected_classes)
        
        # Publish the classes
        msg = String()
        msg.data = detected_classes
        self.class_pub.publish(msg)

        if oneshot:
            self.oneshot_pub.publish(msg)

        return detected_classes

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    observer = Observer()

    try:
        rclpy.spin(observer)

    except KeyboardInterrupt:
        observer.get_logger().info("Shutting down Observer.")
        
    finally:
        observer.destroy_node()


if __name__ == "__main__":
    main()