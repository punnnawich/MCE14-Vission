import socket
import time
import struct

# ==========================================
# การตั้งค่า (Configuration)
# ==========================================
ESP32_IP = "YOUR_ESP32_IP"  # <--- เอารหัส IP ที่ได้จาก ESP32 มาใส่ตรงนี้
ESP32_PORT = 12345

class UDPSender:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # ตัวแปรสำหรับนับลำดับ Packet (Sequence Number)
        self.packet_seq = 0 

    def send_data_binary(self, x, y, z):
        """
        ส่งพิกัดพร้อม Sequence Number (แพ็กเป็น Binary 16 bytes)
        '<Ifff' หมายถึง: 
        - < (Little Endian)
        - I (unsigned int, 4 bytes) สำหรับ Sequence Number
        - f (float, 4 bytes) สำหรับ x
        - f (float, 4 bytes) สำหรับ y
        - f (float, 4 bytes) สำหรับ z
        รวมเป็น 16 bytes
        """
        data = struct.pack('<Ifff', self.packet_seq, float(x), float(y), float(z))
        self.sock.sendto(data, (self.ip, self.port))
        
        self.packet_seq += 1 # เพิ่มลำดับทุกครั้งที่ส่ง

    def close(self):
        self.sock.close()

if __name__ == "__main__":
    sender = UDPSender(ESP32_IP, ESP32_PORT)
    print(f"🚀 เริ่มส่ง UDP ไปที่ {ESP32_IP}:{ESP32_PORT}")
    
    try:
        x, y, z = 0.0, 0.0, 0.0
        while True:
            # จำลองข้อมูล
            x += 0.1; y += 0.2; z += 0.3
            
            # ส่งข้อมูลไป ESP32
            sender.send_data_binary(x, y, z)
            
            # **ข้อควรระวัง:** ไม่ควรส่งเร็วเกินไป (เช่น ไม่มี delay เลย) 
            # เพราะจะเกิด Data Flood ทำลาย Buffer ของ ESP32
            # ควรตั้งค่า Delay ให้ใกล้เคียงกับ Camera FPS (เช่น 30 FPS = ~0.033s)
            time.sleep(1/30.0) 
            
    except KeyboardInterrupt:
        print("\nหยุดการทำงาน")
    finally:
        sender.close()
