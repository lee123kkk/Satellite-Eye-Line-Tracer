import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class VisionTrackerNode(Node):
    def __init__(self):
        super().__init__('vision_tracker_node')
        
        # 1. Gazebo 원본 이미지 구독
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
            
        # 2. 분석 결과가 그려진 이미지를 발행할 퍼블리셔 생성
        self.publisher = self.create_publisher(Image, '/camera/image_tracked', 10)
        
        self.bridge = CvBridge()
        
        # ArUco 사전 및 파라미터 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.get_logger().info("비전 트래커 노드가 시작되었습니다. (결과 화면은 rqt_image_view로 확인하세요)")

    def image_callback(self, msg):
        try:
            # ROS 이미지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"이미지 변환 오류: {e}")
            return

        corners, ids, rejected = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is not None and len(ids) > 0:
            c = corners[0][0]
            
            center_x = int(np.mean(c[:, 0]))
            center_y = int(np.mean(c[:, 1]))

            front_x = (c[0][0] + c[1][0]) / 2.0
            front_y = (c[0][1] + c[1][1]) / 2.0
            
            dx = front_x - center_x
            dy = front_y - center_y
            
            angle_rad = math.atan2(dy, dx)
            # angle_deg = math.degrees(angle_rad)
            
            # 이미지에 마커 윤곽선과 중심점 그리기
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)
            
            # 로봇의 전진 방향을 보여주는 초록색 선 그리기
            end_x = int(center_x + math.cos(angle_rad) * 50)
            end_y = int(center_y + math.sin(angle_rad) * 50)
            cv2.line(cv_image, (center_x, center_y), (end_x, end_y), (0, 255, 0), 2)

        # 3. 그림이 그려진 OpenCV 이미지를 다시 ROS 메시지로 변환하여 발행
        try:
            tracked_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            self.publisher.publish(tracked_msg)
        except Exception as e:
            self.get_logger().error(f"결과 이미지 발행 오류: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()