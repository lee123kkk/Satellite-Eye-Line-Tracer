import cv2
import cv2.aruco as aruco
import numpy as np
import socket
import time
import datetime
import threading

# ROS 2 관련 라이브러리 추가
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# [설정]
HOST = '0.0.0.0'
PORT = 10000

# 파란색 라인 인식을 위한 HSV 범위
BLUE_LOW  = np.array([85, 20, 20])
BLUE_HIGH = np.array([145, 255, 255])
BLUE_FILTER_LOW = np.array([85, 10, 180])
BLUE_FILTER_HIGH = np.array([145, 45, 255])

class RobotNavigator(Node):
    def __init__(self):
        # ROS 2 노드 초기화
        super().__init__('robot_navigator_node')
        
        # 기존 카메라 설정 및 ROS 2 CvBridge 설정
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # start_cam_node.py에서 보낸 이미지를 받는 Subscriber 생성
        self.image_sub = self.create_subscription(
            Image,
            'webcam_image',
            self.image_callback,
            10
        )

        self.width = 800
        self.height = 600
        
        # [삭제 완료] 영상 저장(VideoWriter) 설정 제거됨

        # Aruco 마커 사전(Dictionary) 설정
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        
        # OpenCV 버전에 따른 검출기 최적화 및 호환성 방어 코드 구현
        self.use_legacy_aruco = False
        if hasattr(aruco, 'ArucoDetector'):
            self.detector = aruco.ArucoDetector(self.aruco_dict, aruco.DetectorParameters())
        else:
            self.use_legacy_aruco = True
            self.parameters = aruco.DetectorParameters_create() if hasattr(aruco, 'DetectorParameters_create') else aruco.DetectorParameters()
        
        # 소켓 통신
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((HOST, PORT))
        self.server_socket.listen(1)
        self.server_socket.setblocking(False)
        
        self.conn = None
        self.wifi_status = "Waiting..."
        self.last_send_time = 0
        
        # 경로 추종 변수 유지
        self.waypoints = [] 
        self.max_waypoints = 20      # 하늘색 선 길이를 2배로 연장
        self.point_spacing = 20      
        self.arrival_threshold = 45  # 다음 점 전환 감도 최적화

        # ROS 2 루프와 독립적으로 GUI 및 제어 루프를 실행하기 위한 스레드 구동
        self.running = True
        self.process_thread = threading.Thread(target=self.main_loop)
        self.process_thread.start()

    def image_callback(self, msg):
        """ start_cam_node.py로부터 이미지 토픽을 수신하여 프레임 업데이트 """
        try:
            # ROS Image 메시지를 OpenCV 이미지(BGR8) 포맷으로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # 기존 opencv_waypoint.py 크기 규격(800x600)에 맞춰 리사이즈 처리
            resized_image = cv2.resize(cv_image, (self.width, self.height), interpolation=cv2.INTER_LINEAR)
            with self.frame_lock:
                self.latest_frame = resized_image
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def draw_rotated_rect(self, img, center, f_vec, s_vec, width, height, color, thickness=2):
        p1 = center + (f_vec * height / 2) - (s_vec * width / 2)
        p2 = center + (f_vec * height / 2) + (s_vec * width / 2)
        p3 = center - (f_vec * height / 2) + (s_vec * width / 2)
        p4 = center - (f_vec * height / 2) - (s_vec * width / 2)
        pts = np.array([p1, p2, p3, p4], np.int32)
        cv2.polylines(img, [pts], True, color, thickness)
        return pts

    def find_next_point_on_line(self, mask, current_p, prev_p, search_radius=40):
        direction = current_p - prev_p
        dist = np.linalg.norm(direction)
        if dist == 0: return None
        unit_dir = direction / dist
        expected_p = current_p + unit_dir * self.point_spacing
        y, x = np.where(mask > 0)
        if len(x) == 0: return None
        dists = np.sqrt((x - expected_p[0])**2 + (y - expected_p[1])**2)
        in_range = dists < search_radius
        if np.any(in_range):
            return np.array([np.mean(x[in_range]), np.mean(y[in_range])])
        return None

    def get_first_point(self, mask, center, f_vec, s_vec, width, offset):
        # 로봇 전방 방향으로 스캔 시작점을 잡고, 좌우 각도를 약간 좁혀 안정적으로 차선 검출
        scan_center = center + f_vec * offset
        search_samples = np.linspace(-width / 3, width / 3, 50)
        found_pixels = []
        for s in search_samples:
            sample_p = scan_center + s_vec * s
            qx, qy = int(sample_p[0]), int(sample_p[1])
            if 0 <= qy < mask.shape[0] and 0 <= qx < mask.shape[1]:
                if mask[qy, qx] > 0: found_pixels.append(sample_p)
        return np.mean(found_pixels, axis=0) if len(found_pixels) > 0 else None

    def process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(cv2.inRange(hsv, BLUE_LOW, BLUE_HIGH), cv2.inRange(hsv, BLUE_FILTER_LOW, BLUE_FILTER_HIGH))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

        # 버전에 맞는 문법으로 마커 검출 안전하게 처리
        if self.use_legacy_aruco:
            if hasattr(aruco, 'detectMarkers'):
                corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            else:
                corners, ids = None, None
        else:
            corners, ids, _ = self.detector.detectMarkers(gray)
            
        steer_value, speed_value = 0.0, 0.0

        if ids is not None:
            marker_data = {ids[i][0]: corners[i][0] for i in range(len(ids))}
            if 0 in marker_data:
                c0 = marker_data[0] 
                robot_center = np.mean(c0, axis=0)
                
                # [수정] 바퀴가 없는 쪽(로봇 전방)을 향하도록 방향 벡터 계산식 정교화
                # ID: 0 마커 하단 모서리(2번, 3번 인덱스)가 바퀴가 없는 "진짜 전방"을 향하도록 벡터 정의를 수정합니다.
                front_mid = (c0[2] + c0[3]) / 2  # 바퀴가 없는 전방
                back_mid = (c0[0] + c0[1]) / 2   # 바퀴가 있는 후방
                forward_vec = (front_mid - back_mid).astype(float)
                forward_vec /= np.linalg.norm(forward_vec)
                side_vec = np.array([-forward_vec[1], forward_vec[0]])

                # 초록색 박스 크기 축소 (120 -> 80)
                robot_pts = self.draw_rotated_rect(frame, robot_center, forward_vec, side_vec, 80, 80, (0, 255, 0), 2)
                cv2.fillPoly(mask, [robot_pts], 0)

                # 경로 생성 (정의된 전방 forward_vec을 기준으로 생성)
                if not self.waypoints:
                    p1 = self.get_first_point(mask, robot_center, forward_vec, side_vec, 160, 110)
                    if p1 is not None:
                        self.waypoints.append(p1)
                        curr, prev = p1, robot_center
                        for _ in range(self.max_waypoints - 1):
                            nxt = self.find_next_point_on_line(mask, curr, prev)
                            if nxt is not None:
                                self.waypoints.append(nxt)
                                prev, curr = curr, nxt
                            else: break

                # Waypoint 갱신
                if self.waypoints:
                    if np.linalg.norm(robot_center - self.waypoints[0]) < self.arrival_threshold:
                        self.waypoints.pop(0)
                        if len(self.waypoints) >= 2:
                            new_p = self.find_next_point_on_line(mask, self.waypoints[-1], self.waypoints[-2])
                            if new_p is not None: self.waypoints.append(new_p)

                # 제어 계산
                if self.waypoints:
                    target_p = self.waypoints[0]
                    virtual_front = robot_center + (forward_vec * 80)
                    error_pixel = np.dot(target_p - virtual_front, side_vec)
                    
                    # [수정] 좌우 흔들림(피싱 현상) 최소화를 위한 부드러운 스케일 조향 설계
                    # 직진 상태와 미세 조향 시의 전송값을 매우 세밀하게 나누어 모터 오버슈팅을 막습니다.
                    if abs(error_pixel) < 15:
                        steer_value = 0.0
                        speed_value = 0.10
                    elif abs(error_pixel) < 35:
                        # 미세 조향구간 (0.25 -> 0.15로 대폭 스무딩 완화)
                        steer_value = -0.15 if error_pixel > 0 else 0.15
                        speed_value = 0.09
                    elif abs(error_pixel) < 65:
                        # 중간 조향구간 (0.32)
                        steer_value = -0.32 if error_pixel > 0 else 0.32
                        speed_value = 0.07
                    else:
                        # 회전 시 급작스럽게 확 꺾이지 않도록 최댓값을 안전하게 제한 (0.6 -> 0.52)
                        steer_value = -0.52 if error_pixel > 0 else 0.52
                        speed_value = 0.04

                    cv2.circle(frame, (int(target_p[0]), int(target_p[1])), 10, (0, 0, 255), 3)
                    for i in range(len(self.waypoints)-1):
                        cv2.line(frame, tuple(self.waypoints[i].astype(int)), tuple(self.waypoints[i+1].astype(int)), (255, 255, 0), 2)

        self.display_ui(frame, steer_value, speed_value)
        return speed_value, steer_value, mask

    def display_ui(self, frame, steer, speed):
        color = (0, 255, 0) if self.conn else (0, 0, 255)
        cv2.putText(frame, f"WiFi: {self.wifi_status}", (20, 40), 1, 1.2, color, 2)
        cv2.putText(frame, f"Send: {speed:.2f} S:{steer:.3f}", (20, 60), 1, 1.2, (0, 255, 0), 2)
        cv2.putText(frame, f"Waypoints: {len(self.waypoints)}", (20, 80), 1, 1.2, (255, 255, 0), 2)

    def main_loop(self):
        """ 기존 run()의 역할을 수행하는 메인 프로세스 루프 """
        rate = self.create_rate(30) # 약 30 FPS 속도로 루프 제어
        while rclpy.ok() and self.running:
            frame = None
            with self.frame_lock:
                if self.latest_frame is not None:
                    frame = self.latest_frame.copy()

            if frame is None:
                time.sleep(0.01)
                continue

            # [삭제 완료] self.out.write(frame) 제거됨
            speed, steer, mask = self.process_frame(frame)

            # 소켓 클라이언트(ESP32) 연결 확인 및 수락
            if self.conn is None:
                try: 
                    self.conn, _ = self.server_socket.accept()
                    self.wifi_status = "CONNECTED"
                except: 
                    pass

            # 소켓 전송 제어 (50ms 주기 제한 유지)
            if self.conn and time.time() - self.last_send_time > 0.05:
                try:
                    self.conn.sendall(f"V,{speed:.3f},{steer:.3f}\n".encode())
                    self.last_send_time = time.time()
                except: 
                    self.conn = None
                    self.wifi_status = "RECONNECTING"

            cv2.imshow("Robot AI Navigator", frame) 
            if cv2.waitKey(1) == 27: 
                break 
            
            try:
                rate.sleep()
            except Exception:
                break

        # 루프 종료 시 정리 작업
        # [삭제 완료] self.out.release() 제거됨
        cv2.destroyAllWindows()
        if self.conn:
            self.conn.close()
        self.server_socket.close()

    def stop(self):
        self.running = False
        if self.process_thread.is_alive():
            self.process_thread.join()

def main(args=None):
    rclpy.init(args=args)
    nav = RobotNavigator()
    
    # 멀티스레드 실행기 적용 (토픽 수신과 소켓/GUI 루프를 병렬 구동)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(nav)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nav.stop()
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()