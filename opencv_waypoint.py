import cv2
import cv2.aruco as aruco
import numpy as np
import socket
import time
import datetime

# [설정]
CAM_INDEX = 0
HOST = '0.0.0.0'
PORT = 10000

# 파란색 라인 인식을 위한 HSV 범위
BLUE_LOW  = np.array([85, 20, 20])
BLUE_HIGH = np.array([145, 255, 255])
BLUE_FILTER_LOW = np.array([85, 10, 180])
BLUE_FILTER_HIGH = np.array([145, 45, 255])

class RobotNavigator:
    def __init__(self):
        self.cap = cv2.VideoCapture(CAM_INDEX)
        self.width = 800
        self.height = 600
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        
        # 영상 저장 설정
        now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"Robot_AI_Navigator_{now}.avi"
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter(filename, fourcc, 20.0, (self.width, self.height))

        # Aruco 마커 설정
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.detector = aruco.ArucoDetector(self.aruco_dict, aruco.DetectorParameters())
        
        # 소켓 통신
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((HOST, PORT))
        self.server_socket.listen(1)
        self.server_socket.setblocking(False)
        
        self.conn = None
        self.wifi_status = "Waiting..."
        self.last_send_time = 0
        
        # 경로 추종 변수 수정
        self.waypoints = [] 
        self.max_waypoints = 20      # 하늘색 선 길이를 2배로 연장[cite: 1]
        self.point_spacing = 20      
        self.arrival_threshold = 45  # 다음 점 전환 감도 최적화[cite: 1]

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
        scan_center = center + f_vec * offset
        search_samples = np.linspace(-width / 2, width / 2, 50)
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

        corners, ids, _ = self.detector.detectMarkers(gray)
        steer_value, speed_value = 0.0, 0.0

        if ids is not None:
            marker_data = {ids[i][0]: corners[i][0] for i in range(len(ids))}
            if 0 in marker_data:
                c0 = marker_data[0] 
                robot_center = np.mean(c0, axis=0)
                front_mid, back_mid = (c0[0] + c0[1]) / 2, (c0[2] + c0[3]) / 2  
                forward_vec = (back_mid - front_mid).astype(float)
                forward_vec /= np.linalg.norm(forward_vec)
                side_vec = np.array([-forward_vec[1], forward_vec[0]])

                # [수정] 초록색 박스 크기 축소 (120 -> 80)[cite: 1]
                robot_pts = self.draw_rotated_rect(frame, robot_center, forward_vec, side_vec, 80, 80, (0, 255, 0), 2)
                cv2.fillPoly(mask, [robot_pts], 0)

                # 경로 생성
                if not self.waypoints:
                    p1 = self.get_first_point(mask, robot_center, forward_vec, side_vec, 200, 100)
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
                    
                    if abs(error_pixel) < 25: steer_value, speed_value = 0.0, 0.12
                    elif abs(error_pixel) < 50: steer_value, speed_value = (-0.25 if error_pixel > 0 else 0.25), 0.07
                    else: steer_value, speed_value = (-0.6 if error_pixel > 0 else 0.6), 0.04

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

    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret: break
            speed, steer, mask = self.process_frame(frame)
            self.out.write(frame)
            if self.conn is None:
                try: self.conn, _ = self.server_socket.accept(); self.wifi_status = "CONNECTED"
                except: pass
            if self.conn and time.time() - self.last_send_time > 0.05:
                try:
                    self.conn.sendall(f"V,{speed:.3f},{steer:.3f}\n".encode())
                    self.last_send_time = time.time()
                except: self.conn = None; self.wifi_status = "RECONNECTING"
            cv2.imshow("Robot AI Navigator", frame) 
            if cv2.waitKey(1) == 27: break 
        self.cap.release(); self.out.release(); cv2.destroyAllWindows()

if __name__ == "__main__":
    nav = RobotNavigator(); nav.run()