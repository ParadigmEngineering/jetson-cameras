import cv2
import rclpy

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('camera_node')
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', qos_profile)
        self.br = CvBridge()

        self.cap = cv2.VideoCapture(self.gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open camera")

    def gstreamer_pipeline(
        self,
        sensor_id=0,
        capture_width=1920,
        capture_height=1080,
        display_width=960,
        display_height=540,
        framerate=60,
        flip_method=0,
    ):
        return (
            "nvarguscamerasrc sensor-id=%d !"
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=true"
            % (
                sensor_id,
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
        )
        

    def stream_video(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                self.publisher_.publish(self.br.cv2_to_imgmsg(frame, encoding='passthrough'))
                self.get_logger().info('Publishing video frame')

def main(args=None):
    rclpy.init(args=args)
    camera_node = ImagePublisher()
    camera_node.stream_video()
    camera_node.cap.release()
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

