import cv2
import depthai as dai
import numpy as np
import torch
from ultralytics import YOLO
import socket
import time
from udp_sender import UDPSender # นำเข้าคลาสส่งข้อมูลไป ESP32

# ค้นหาว่ามี GPU ให้ใช้หรือไม่
DEVICE = 'cuda:0' if torch.cuda.is_available() else 'cpu'
USE_HALF = (DEVICE == 'cuda:0') # ใช้ FP16 เมื่อเป็น GPU เพื่อให้ไวที่สุด

class BallTrackerKF:
    def __init__(self):
        self.state = np.zeros((6, 1)) # [x, y, z, vx, vy, vz]
        self.P = np.eye(6) * 1000.0
        self.R = np.diag([5.0, 5.0, 15.0])
        self.Q = np.eye(6) * 0.1
        self.Q[3:, 3:] *= 10.0
        
        # Pre-allocate matrices (Optimization for speed)
        self.F = np.eye(6)
        self.B = np.zeros((6, 1))
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1.0; self.H[1, 1] = 1.0; self.H[2, 2] = 1.0
        self.I = np.eye(6)
        
        self.last_time = time.time()
        self.is_initialized = False

    def process(self, measurement=None):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt > 1.0:
            self.is_initialized = False
            
        if not self.is_initialized:
            if measurement is not None:
                self.state[:3] = np.array(measurement).reshape((3, 1))
                self.state[3:] = 0
                self.is_initialized = True
            return self.state
            
        # Predict
        self.F[0, 3] = dt; self.F[1, 4] = dt; self.F[2, 5] = dt
        self.B[1, 0] = -0.5 * 981 * dt**2
        self.B[4, 0] = -981 * dt
        
        self.state = np.dot(self.F, self.state) + self.B
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        
        # Update
        if measurement is not None:
            z = np.array(measurement).reshape((3, 1))
            y = z - np.dot(self.H, self.state)
            S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
            K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
            self.state = self.state + np.dot(K, y)
            self.P = np.dot(self.I - np.dot(K, self.H), self.P)
            
        return self.state
        
    def predict_landing(self):
        if not self.is_initialized:
            return None, None
            
        x, y, z, vx, vy, vz = self.state.flatten()
        a = -490.5 # -0.5 * g
        b = vy
        c = y
        
        discriminant = b**2 - 4*a*c
        if discriminant < 0:
            return None, None
            
        t1 = (-b + np.sqrt(discriminant)) / (2*a)
        t2 = (-b - np.sqrt(discriminant)) / (2*a)
        t_land = max(t1, t2)
        
        if t_land < 0:
            return None, None
            
        x_land = x + vx * t_land
        z_land = z + vz * t_land
        return float(x_land), float(z_land)

# ตั้งค่า UDP Socket สำหรับส่งข้อมูล 3D ไปวาดกราฟ
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ตั้งค่า UDP Sender สำหรับส่งไป ESP32
# อย่าลืมเปลี่ยน IP ตรงนี้ให้ตรงกับที่แสดงใน Serial Monitor ของ ESP32
ESP32_IP = "192.168.1.XXX" 
ESP32_PORT = 12345
esp32_sender = UDPSender(ESP32_IP, ESP32_PORT)

# 1. โหลดโมเดล YOLOv8 และบังคับใช้ GPU ถ้ารองรับ
model_path = r"C:\Users\punna\OneDrive\Documents\runs\detect\redball_model\weights\best.pt"
model = YOLO(model_path)
model.to(DEVICE)

# 2. ตั้งค่า DepthAI (OAK-D Lite) Pipeline
pipeline = dai.Pipeline()

# 2.1 กล้องสี RGB
camRgb = pipeline.create(dai.node.ColorCamera)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setIspScale(1, 3)  # แปลงขนาดเป็น 1920//3 = 640, 1080//3 = 360
camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)

xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutRgb.setStreamName("rgb")
camRgb.isp.link(xoutRgb.input)

# 2.2 กล้องขาวดำซ้ายและขวา (สำหรับ Depth)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)

monoRight = pipeline.create(dai.node.MonoCamera)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)

# 2.3 ตัวรวมสัญญาณ 3 มิติ (Stereo Depth)
stereo = pipeline.create(dai.node.StereoDepth)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)

# จัดตำแหน่งภาพ 3 มิติให้ตรงกับกล้องสี และบังคับขนาดภาพให้ตรงกัน
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
stereo.setOutputSize(camRgb.getIspWidth(), camRgb.getIspHeight())

monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutDepth.setStreamName("depth")
stereo.depth.link(xoutDepth.input)


# ฟังก์ชัน: หาระยะ Z
def get_distance_from_center(depth_map, x1, y1, x2, y2):
    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2
    
    # วัดแค่บริเวณตรงกลางเป้าหมาย (ตีกรอบความกว้าง 20x20 พิกเซล)
    roi_x1 = max(0, center_x - 10)
    roi_y1 = max(0, center_y - 10)
    roi_x2 = min(depth_map.shape[1], center_x + 10)
    roi_y2 = min(depth_map.shape[0], center_y + 10)
    
    roi = depth_map[roi_y1:roi_y2, roi_x1:roi_x2]
    valid_depths = roi[roi > 0]
    
    if len(valid_depths) > 0:
        return np.median(valid_depths) # คืนค่า Z เป็น มิลลิเมตร
    return 0


# 3. รัน OAK-D Lite
with dai.Device(pipeline) as device:
    print("เริ่มเชื่อมต่อกล้อง กำลังดึงค่า Camera Intrinsics...")
    
    # อ่านค่า Calibration เพื่อดึงตัวแปรมาคำนวณแกน X และ Y
    calibData = device.readCalibration()
    
    # ขอค่า Intrinsics ของกล้องสี CAM_A ที่ขนาด 640x360
    intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, camRgb.getIspWidth(), camRgb.getIspHeight())
    
    # ดึงค่าจุดโฟกัสและจุดกึ่งกลาง (Focal Length / Principal Point)
    fx = intrinsics[0][0]
    fy = intrinsics[1][1]
    cx = intrinsics[0][2]
    cy = intrinsics[1][2]
    
    print(f"ค่าของเลนส์กล้อง: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}")
    print("เริ่มแสดงผล (กด 'q' เพื่อออก)")
    
    # maxSize=1 ดึงเฉพาะ Frame ล่าสุด ตัด Delay ที่เกิดจาก Buffer ของกล้อง
    qRgb = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
    qDepth = device.getOutputQueue(name="depth", maxSize=1, blocking=False)
    
    kf = BallTrackerKF()
    
    while True:
        inRgb = qRgb.get()
        inDepth = qDepth.tryGet()
        
        if inRgb is not None:
            frame = inRgb.getCvFrame()
            
            depth_frame = None
            if inDepth is not None:
                depth_frame = inDepth.getFrame()
            
            # YOLOv8 (Acceleration with FP16 + GPU Core)
            results = model.predict(frame, conf=0.5, verbose=False, device=DEVICE, half=USE_HALF)
            annotated_frame = frame.copy()
            
            measured_xyz = None
            
            if len(results[0].boxes) > 0:
                for box in results[0].boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    
                    label_text = "N/A"
                    if depth_frame is not None:
                        Z_mm = get_distance_from_center(depth_frame, x1, y1, x2, y2)
                        
                        if Z_mm > 0:
                            # คำนวณแกน X: (x - cx) * Z / fx
                            X_mm = (center_x - cx) * Z_mm / fx
                            # คำนวณแกน Y: (y - cy) * Z / fy
                            Y_mm = (center_y - cy) * Z_mm / fy
                            
                            # ความสูงกล้องจากพื้น (อ้างอิงจาก Y ที่วัดได้ตอนบอลอยู่บนพื้น)
                            CAMERA_HEIGHT_CM = 121.4
                            
                            # แปลงเป็นหน่วย เซนติเมตร (cm) 
                            X_cm = X_mm / 10.0
                            Y_cam_cm = Y_mm / 10.0
                            Z_cm = Z_mm / 10.0
                            
                            # ปรับให้พิกัด Y=0 คือพื้น (ถ้าลูกบอลลอยขึ้น Y จะเป็นบวก)
                            Y_cm = CAMERA_HEIGHT_CM - Y_cam_cm
                            
                            if measured_xyz is None:
                                measured_xyz = [X_cm, Y_cm, Z_cm]
                                
                            label_text = f"X: {X_cm:.1f}cm, Y: {Y_cm:.1f}cm, Z: {Z_cm:.1f}cm"
                    
                    # วาดสีกล่อง
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    
                    # วาดจุดกึ่งกลาง (Center Point)
                    cv2.circle(annotated_frame, (center_x, center_y), 5, (0, 255, 0), -1)
                    
                    # ใส่ตัวหนังสือพิกัด 
                    cv2.putText(annotated_frame, label_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # อัพเดต Kalman Filter 1 ครั้งต่อ 1 เฟรม
            kf.process(measured_xyz)
            landing_pt = kf.predict_landing()
            
            # ส่งข้อมูลไปยัง plot_3d.py รวมพิกัดปัจจุบันและจุดตกที่พยากรณ์ได้ รวมถึงความเร็วจาก KF
            if measured_xyz is not None:
                vx, vy, vz = kf.state[3:].flatten()
                if landing_pt[0] is not None:
                    msg = f"{measured_xyz[0]:.2f},{measured_xyz[1]:.2f},{measured_xyz[2]:.2f},{landing_pt[0]:.2f},{landing_pt[1]:.2f},{vx:.2f},{vy:.2f},{vz:.2f}"
                else:
                    msg = f"{measured_xyz[0]:.2f},{measured_xyz[1]:.2f},{measured_xyz[2]:.2f},None,None,{vx:.2f},{vy:.2f},{vz:.2f}"
                try:
                    sock.sendto(msg.encode('utf-8'), (UDP_IP, UDP_PORT))
                    
                    # 🚀 ส่งข้อมูลจุดตก (Landing Point) ไปที่ ESP32
                    if landing_pt[0] is not None:
                        esp32_sender.send_data_binary(landing_pt[0], 0.0, landing_pt[1])
                    
                except Exception:
                    pass
            
            if landing_pt[0] is not None:
                lx, lz = landing_pt
                cv2.putText(annotated_frame, f"Pred Land X:{lx:.1f} Z:{lz:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                kx, ky, kz = kf.state[:3].flatten()
                cv2.putText(annotated_frame, f"KF X:{kx:.1f} Y:{ky:.1f} Z:{kz:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            
            cv2.imshow("Red Ball 3D Map (X, Y, Z)", annotated_frame)
            
            if depth_frame is not None:
                if depth_frame.shape[0] > 0 and depth_frame.shape[1] > 0:
                    depth_rendered = cv2.normalize(depth_frame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
                    depth_rendered = cv2.equalizeHist(depth_rendered)
                    depth_rendered = cv2.applyColorMap(depth_rendered, cv2.COLORMAP_JET)
                    cv2.imshow("Depth Heatmap", depth_rendered)
            
        if cv2.waitKey(1) == ord('q'):
            break

cv2.destroyAllWindows()
esp32_sender.close()
