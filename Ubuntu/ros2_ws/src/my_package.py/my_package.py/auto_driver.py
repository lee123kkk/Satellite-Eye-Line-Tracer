import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class AutoDriver(Node):
    def __init__(self):
        super().__init__('auto_driver_node')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_img = self.create_publisher(Image, '/camera/image_tracked', 10)
        self.bridge = CvBridge()
        
        self.state = 'SEARCHING' 
        self.locked_tx = 0       
        self.locked_ty = 0       
        
        self.get_logger().info("자기 몸체 필터링 및 디버깅 로그가 추가된 자율주행이 시작되었습니다.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8').copy()
        except Exception:
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 1. 색상 마스크 생성
        mask_white = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 50, 255]))
        mask_green = cv2.inRange(hsv, np.array([40, 50, 50]), np.array([80, 255, 255]))
        mask_red1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
        mask_red2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        M_green = cv2.moments(mask_green)
        M_red = cv2.moments(mask_red)
        cmd = Twist()

        if M_green["m00"] > 0 and M_red["m00"] > 0:
            bx = int(M_green["m10"] / M_green["m00"])
            by = int(M_green["m01"] / M_green["m00"])

            fx = int(M_red["m10"] / M_red["m00"])
            fy = int(M_red["m01"] / M_red["m00"])

            rx = int((bx + fx) / 2)
            ry = int((by + fy) / 2)
            robot_angle = math.atan2(fy - by, fx - bx)

            # --- 핵심 해결책: 로봇 자신의 몸체(하얀색 부품)를 선으로 착각하지 않도록 까맣게 지워버림 ---
            cv2.circle(mask_white, (rx, ry), 35, 0, -1) 

            white_y, white_x = np.where(mask_white > 0)
            
            if len(white_x) == 0:
                if self.state != 'SEARCHING':
                    self.state = 'SEARCHING'
                    self.get_logger().info("화면에 라인이 없습니다! 제자리 회전하며 탐색합니다.")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5
                self.publisher_cmd.publish(cmd)
                return

            target_x, target_y = rx, ry

            # ================= [상태 1: 탐색] =================
            if self.state == 'SEARCHING':
                distances = np.sqrt((white_x - rx)**2 + (white_y - ry)**2)
                closest_idx = np.argmin(distances)
                
                # 목표 고정
                self.locked_tx = int(white_x[closest_idx])
                self.locked_ty = int(white_y[closest_idx])
                self.state = 'APPROACHING'
                
                # 디버깅 로그 출력
                self.get_logger().info(f"[SEARCHING] 목표물 포착! 로봇:({rx}, {ry}) -> 목표:({self.locked_tx}, {self.locked_ty})")

            # ================= [상태 2: 목표로 접근] =================
            elif self.state == 'APPROACHING':
                target_x, target_y = self.locked_tx, self.locked_ty
                cv2.circle(cv_image, (target_x, target_y), 12, (255, 0, 0), -1) 
                
                # 내가 쫓고 있는 고정 목표물과의 실제 거리 계산
                dist_to_locked = math.sqrt((self.locked_tx - rx)**2 + (self.locked_ty - ry)**2)
                
                if dist_to_locked < 25:
                    self.state = 'TRACKING'
                    self.get_logger().info(f"[ARRIVED] 도달 완료! 현재:({rx}, {ry}) / 목표였던 곳:({self.locked_tx}, {self.locked_ty})")

            # ================= [상태 3: 라인 트래킹] =================
            elif self.state == 'TRACKING':
                distances = np.sqrt((white_x - rx)**2 + (white_y - ry)**2)
                lookahead_dist = 60
                valid_idx = np.where((distances > lookahead_dist - 20) & (distances < lookahead_dist + 20))[0]

                target_found = False
                if len(valid_idx) > 0:
                    best_penalty = float('inf')
                    for idx in valid_idx:
                        px, py = white_x[idx], white_y[idx]
                        angle_to_pt = math.atan2(py - ry, px - rx)
                        diff = math.atan2(math.sin(angle_to_pt - robot_angle), math.cos(angle_to_pt - robot_angle))
                        
                        if abs(diff) < math.radians(135):
                            penalty = abs(diff)
                            if diff > 0: 
                                penalty += 0.001
                            if penalty < best_penalty:
                                best_penalty = penalty
                                target_x, target_y = px, py
                                target_found = True
                
                if not target_found:
                    self.state = 'SEARCHING'
                    self.get_logger().info(f"[LOST] 경로 이탈! 위치:({rx}, {ry}) - 다시 탐색합니다.")
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.publisher_cmd.publish(cmd)
                    return

            # 시각화 드로잉
            cv2.circle(cv_image, (bx, by), 5, (0, 255, 0), -1)
            cv2.circle(cv_image, (fx, fy), 5, (0, 0, 255), -1)
            cv2.line(cv_image, (bx, by), (fx, fy), (255, 255, 0), 2)
            if self.state in ['APPROACHING', 'TRACKING']:
                cv2.circle(cv_image, (target_x, target_y), 8, (0, 255, 255), -1)
                cv2.line(cv_image, (rx, ry), (target_x, target_y), (0, 255, 0), 2)

            # --- 직진/회전 분리 제어 ---
            if self.state in ['APPROACHING', 'TRACKING']:
                angle_to_target = math.atan2(target_y - ry, target_x - rx)
                error_angle = math.atan2(math.sin(angle_to_target - robot_angle), math.cos(angle_to_target - robot_angle))

                tolerance = math.radians(15)

                if abs(error_angle) > tolerance:
                    cmd.linear.x = 0.0
                    cmd.angular.z = -0.8 if error_angle > 0 else 0.8
                else:
                    cmd.linear.x = 0.15
                    cmd.angular.z = 0.0

        self.publisher_cmd.publish(cmd)

        try:
            tracked_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            self.publisher_img.publish(tracked_msg)
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = AutoDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()