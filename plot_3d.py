import socket
import matplotlib.pyplot as plt
import numpy as np

# กำหนด IP และ Port สำหรับรับข้อมูล UDP
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

# ตั้งค่า Matplotlib ให้อัปเดตแบบ Real-time
plt.ion()
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

# กำหนดขอบเขตและป้ายกำกับแกนของกราฟตั้งแต่แรก (ค่าเริ่มต้นเบื้องต้น)
ax.set_xlim([-200, 200])
ax.set_ylim([0, 400])    # ความลึก (Z เดิม) จะพล็อตไว้แกน Y ของกราฟ
ax.set_zlim([-10, 300])  # ความสูง (Y ใหม่) กำหนดให้อยู่ในช่วง -10 ถึง 300 ซม.
ax.set_xlabel('X (Width - cm)')
ax.set_ylabel('Z (Depth - cm)')
ax.set_zlabel('Y (Height above floor - cm)')
ax.set_title('Real-time 3D Ball Tracking')

# เตรียมตัวแปรอ็อบเจ็กต์กราฟสำหรับอัปเดตข้อมูล
# scatter_history = สำหรับหาง, scatter_latest = สำหรับจุดล่าสุด, plot_line = สำหรับเส้นเชื่อม
scatter_history = ax.plot([], [], [], marker='o', linestyle='None', c='gray', alpha=0.5)[0]
scatter_latest = ax.plot([], [], [], marker='o', linestyle='None', c='red', markersize=10)[0]
plot_line = ax.plot([], [], [], c='blue', alpha=0.5)[0]
scatter_landing = ax.plot([], [], [], marker='X', linestyle='None', c='green', markersize=12, label='Predicted Landing')[0]
plot_curve = ax.plot([], [], [], c='orange', linestyle='--', linewidth=2, label='Predicted Trajectory')[0]
ax.legend()

# เก็บประวัติตำแหน่ง (History) ความยาวของหาง (รอยลาก)
max_points = 20
xs, ys, zs = [], [], []

print(f"กำลังรอรับข้อมูล 3D ทางพอร์ต {UDP_PORT}...")
print("คำแนะนำ: เปิด run script นี้พร้อมกับ track_3d.py")
print("กด Ctrl+C ที่ Terminal เพื่อออก หรือปิดหน้าต่างกราฟ")

try:
    while True:
        data = None
        # อ่านข้อมูลล่าสุดใน Buffer ให้หมดเพื่อไม่ให้กราฟหน่วง (delay)
        while True:
            try:
                packet, addr = sock.recvfrom(1024)
                data = packet
            except BlockingIOError:
                break
            except Exception as e:
                pass
        
        if data:
            decoded = data.decode('utf-8')
            parts = decoded.split(',')
            if len(parts) >= 3:
                try:
                    raw_x = float(parts[0])
                    raw_y = float(parts[1])
                    raw_z = float(parts[2])
                    
                    # เลื่อนแกนให้แสดงผลบน 3D plot ได้สมจริง (เอาพื้นไว้ล่างสุด)
                    # Matplotlib X = แกนขวางซ้ายขวา (raw_x)
                    # Matplotlib Y = แกนลึกเข้าไป (raw_z)  **สลับ**
                    # Matplotlib Z = แกนตั้งสูงต่ำพื้น (raw_y) **สลับ**
                    
                    # เพิ่มข้อมูลใหม่
                    xs.append(raw_x)
                    ys.append(raw_z) # ความลึก
                    zs.append(raw_y) # ความสูงจากพื้น
                    
                    pred_x, pred_z = None, None
                    if len(parts) >= 8 and parts[3] != 'None':
                        pred_x = float(parts[3])
                        pred_z = float(parts[4])
                        vx = float(parts[5])
                        vy = float(parts[6])
                        vz = float(parts[7])
                        
                        scatter_landing.set_data([pred_x], [pred_z])
                        scatter_landing.set_3d_properties([0.0]) # พื้นคือ 0
                        
                        # คำนวณเส้นโค้งโปรเจคไทล์จากวิถีปัจจุบันไปจนถึงจุดตก
                        a = -490.5 # แรงโน้มถ่วง -0.5*g โดย g=981 cm/s^2
                        discriminant = vy**2 - 4*a*raw_y
                        if discriminant >= 0:
                            t1 = (-vy + np.sqrt(discriminant)) / (2*a)
                            t2 = (-vy - np.sqrt(discriminant)) / (2*a)
                            t_land = max(t1, t2)
                            if t_land > 0:
                                t_arr = np.linspace(0, t_land, num=30)
                                curve_x = raw_x + vx * t_arr
                                curve_y = raw_y + vy * t_arr + a * t_arr**2
                                curve_z = raw_z + vz * t_arr
                                
                                plot_curve.set_data(curve_x, curve_z)
                                plot_curve.set_3d_properties(curve_y)
                            else:
                                plot_curve.set_data([], [])
                                plot_curve.set_3d_properties([])
                        else:
                            plot_curve.set_data([], [])
                            plot_curve.set_3d_properties([])
                            
                    elif len(parts) >= 5 and parts[3] != 'None':
                        pred_x = float(parts[3])
                        pred_z = float(parts[4])
                        scatter_landing.set_data([pred_x], [pred_z])
                        scatter_landing.set_3d_properties([0.0])
                        plot_curve.set_data([], [])
                        plot_curve.set_3d_properties([])
                    else:
                        scatter_landing.set_data([], [])
                        scatter_landing.set_3d_properties([])
                        plot_curve.set_data([], [])
                        plot_curve.set_3d_properties([])
                        
                    # ลบข้อมูลเก่าถ้าเกินจำนวนที่กำหนด
                    if len(xs) > max_points:
                        xs.pop(0)
                        ys.pop(0)
                        zs.pop(0)
                    
                    # อัปเดตข้อมูลให้กับอ็อบเจ็กต์กราฟแทนการล้าง (clear) แกน
                    if len(xs) > 1:
                        scatter_history.set_data(xs[:-1], ys[:-1])
                        scatter_history.set_3d_properties(zs[:-1])
                    else:
                        scatter_history.set_data([], [])
                        scatter_history.set_3d_properties([])
                    
                    if len(xs) > 0:
                        scatter_latest.set_data([xs[-1]], [ys[-1]])
                        scatter_latest.set_3d_properties([zs[-1]])
                    
                        plot_line.set_data(xs, ys)
                        plot_line.set_3d_properties(zs)
                        
                        current_max_x = max(map(abs, xs))
                        current_max_y = max(ys)
                        if pred_x is not None:
                            current_max_x = max(current_max_x, abs(pred_x))
                            current_max_y = max(current_max_y, pred_z)
                            
                        # อัปเดตขอบเขตแกนแบบ Dynamic ให้อัตโนมัติ (ไม่ให้หนีออกนอกจอ)
                        max_x = max(current_max_x * 1.5, 200) # กะระยะเผื่อ 50%
                        max_y = max(current_max_y * 1.5, 400) # ความลึกของห้อง (พล็อตบนแกน Y)
                        
                        ax.set_xlim([-max_x, max_x])
                        ax.set_ylim([0, max_y])
                        # แกน Z (ความสูง) ให้คงที่ไว้ที่ -10 ถึง 300 ตามที่กำหนด
                    
                except ValueError:
                    pass
        
        # วาดกราฟใหม่เฉพาะส่วนที่เปลี่ยน
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        
        # เช็คว่าผู้ใช้กดปิดหน้าต่างกราฟไปหรือยัง
        if not plt.fignum_exists(fig.number):
            print("ปิดหน้าต่างกราฟแล้ว กำลังจบการทำงาน...")
            break
            
except KeyboardInterrupt:
    print("\nหยุดการทำงานโดยผู้ใช้ (Ctrl+C)")
finally:
    sock.close()
    plt.ioff()
    plt.close('all')
