import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class WaypointDriver2(Node):
    def __init__(self):
        super().__init__('waypoint_driver_node2')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_img = self.create_publisher(Image, '/camera/image_tracked', 10)
        self.bridge = CvBridge()
        
        self.state = 'SEARCHING' 
        self.locked_tx = 0       
        self.locked_ty = 0       
        
        self.path_queue = []
        self.last_target_x = 0
        self.last_target_y = 0

        self.get_logger().info("V2 튜닝 완료: 더욱 촘촘해진 초정밀 11-Step 웨이포인트 가동!")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8').copy()
        except Exception:
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        lower_blue = np.array([100, 100, 50])
        upper_blue = np.array([140, 255, 255])
        mask_track = cv2.inRange(hsv, lower_blue, upper_blue)

        mask_green = cv2.inRange(hsv, np.array([40, 50, 50]), np.array([80, 255, 255]))
        mask_red = cv2.bitwise_or(
            cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255])),
            cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
        )

        M_green = cv2.moments(mask_green)
        M_red = cv2.moments(mask_red)
        cmd = Twist()

        if M_green["m00"] > 0 and M_red["m00"] > 0:
            bx = int(M_green["m10"] / M_green["m00"])
            by = int(M_green["m01"] / M_green["m00"])
            fx = int(M_red["m10"] / M_red["m00"])
            fy = int(M_red["m01"] / M_red["m00"])

            rx = int(bx + (fx - bx) * (5/9))
            ry = int(by + (fy - by) * (5/9))
            robot_angle = math.atan2(fy - by, fx - bx)
            
            target_x, target_y = rx, ry 

            cv2.circle(cv_image, (rx, ry), 28, (0, 0, 255), 2)
            track_y, track_x = np.where(mask_track > 0)

            if len(track_x) == 0 and len(self.path_queue) == 0:
                self.state = 'SEARCHING'
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5
            else:
                if self.state == 'SEARCHING':
                    self.path_queue.clear() 
                    distances = np.sqrt((track_x - rx)**2 + (track_y - ry)**2)
                    closest_idx = np.argmin(distances)
                    self.locked_tx = int(track_x[closest_idx])
                    self.locked_ty = int(track_y[closest_idx])
                    self.state = 'APPROACHING'

                elif self.state == 'APPROACHING':
                    self.path_queue.clear()
                    target_x, target_y = self.locked_tx, self.locked_ty
                    cv2.circle(cv_image, (target_x, target_y), 15, (255, 0, 0), -1) 
                    
                    if math.sqrt((target_x - rx)**2 + (target_y - ry)**2) < 40:
                        self.state = 'TRACKING'
                        self.last_target_x = rx
                        self.last_target_y = ry

                elif self.state == 'TRACKING':
                    temp_mask = mask_track.copy()
                    cv2.circle(temp_mask, (rx, ry), 28, 0, -1)
                    
                    # [V2 튜닝] 지우개 크기를 12로 더욱 축소
                    erase_radius = 12
                    for px, py in self.path_queue:
                        cv2.circle(temp_mask, (px, py), erase_radius, 0, -1)

                    if len(self.path_queue) == 0:
                        distances = np.sqrt((track_x - rx)**2 + (track_y - ry)**2)
                        # [V2 튜닝] 첫 점 탐색 거리를 더 가깝게 (20~40)
                        ring_mask = (distances > 20) & (distances < 40)
                        ring_idx = np.where(ring_mask)[0]
                        if len(ring_idx) > 0:
                            dists_to_last = np.sqrt((track_x[ring_idx] - self.last_target_x)**2 + (track_y[ring_idx] - self.last_target_y)**2)
                            best_idx = ring_idx[np.argmin(dists_to_last)]
                            cx, cy = int(track_x[best_idx]), int(track_y[best_idx])
                            self.path_queue.append((cx, cy))
                            cv2.circle(temp_mask, (cx, cy), erase_radius, 0, -1)

                    # [V2 튜닝] 탐색 반경을 20으로 축소하여 촘촘하게 찍음
                    search_radius = 20  
                    while len(self.path_queue) > 0 and len(self.path_queue) < 11:
                        cx, cy = self.path_queue[-1]
                        ty, tx = np.where(temp_mask > 0)
                        if len(tx) > 0:
                            dists = np.sqrt((tx - cx)**2 + (ty - cy)**2)
                            in_window = np.where(dists < search_radius)[0] 
                            if len(in_window) > 0:
                                nx = int(np.mean(tx[in_window]))
                                ny = int(np.mean(ty[in_window]))
                                self.path_queue.append((nx, ny))
                                cv2.circle(temp_mask, (nx, ny), erase_radius, 0, -1)
                            else:
                                break
                        else:
                            break

                    if len(self.path_queue) > 0:
                        target_x, target_y = self.path_queue[0]

                        # [V2 튜닝] 점이 가까워졌으니 도달 판정도 10으로 축소
                        if math.hypot(target_x - rx, target_y - ry) < 10:
                            self.last_target_x, self.last_target_y = target_x, target_y
                            self.path_queue.pop(0)
                            if len(self.path_queue) > 0:
                                target_x, target_y = self.path_queue[0]
                    else:
                        self.state = 'SEARCHING'

                    for i, pt in enumerate(self.path_queue):
                        cv2.circle(cv_image, pt, 4, (0, 255, 255), -1)
                        if i > 0:
                            cv2.line(cv_image, self.path_queue[i-1], pt, (0, 255, 255), 2)

                    cv2.circle(cv_image, (target_x, target_y), 8, (0, 255, 0), -1)
                    cv2.line(cv_image, (rx, ry), (target_x, target_y), (0, 255, 0), 2)

            cv2.circle(cv_image, (bx, by), 5, (0, 255, 0), -1) 
            cv2.circle(cv_image, (fx, fy), 5, (0, 0, 255), -1) 

            if self.state in ['APPROACHING', 'TRACKING']:
                angle_to_target = math.atan2(target_y - ry, target_x - rx)
                error_angle = math.atan2(math.sin(angle_to_target - robot_angle), math.cos(angle_to_target - robot_angle))

                if abs(error_angle) > math.radians(10):
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
    node = WaypointDriver2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
