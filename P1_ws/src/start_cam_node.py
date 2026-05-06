import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CameraNode(Node):
    def __init__(self):
        super().__init__('webcam_node')
        self.publisher_ = self.create_publisher(Image, 'webcam_image', 1)
        self.timer = self.create_timer(0.033, self.timer_callback)

        # 1. 0번 카메라 지정 및 V4L2 드라이버 사용
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

        # 2. MJPG 압축 포맷 및 해상도 강제 지정 (WSL2 병목 현상 해결)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_NEAREST)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
        else:
            self.get_logger().error('Failed to capture image')

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        # 3. 종료 시 발생하는 빨간색 에러 방지용 안전장치
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()