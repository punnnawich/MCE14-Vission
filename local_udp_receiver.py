import socket
import struct

# ==========================================
# การตั้งค่า (Configuration)
# ==========================================
LISTEN_IP = "127.0.0.1"  
LISTEN_PORT = 12345

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    
    print(f"📡 ตัวจำลอง ESP32 เปิดรับข้อมูล UDP อยู่ที่ {LISTEN_IP}:{LISTEN_PORT}")
    print("รอรับข้อมูล...")
    
    last_seq = -1
    
    try:
        while True:
            data, addr = sock.recvfrom(1024)
            
            # ตรวจสอบขนาดข้อมูลว่าเท่ากับ 16 bytes หรือไม่ (ตามแพ็คเกจที่ตัวส่งส่งมา)
            if len(data) == 16:
                # แกะข้อมูล (Unpack) 
                # <Ifff = Little Endian, unsigned int, float, float, float
                seq_num, x, y, z = struct.unpack('<Ifff', data)
                
                # ป้องกัน out-of-order packets
                if seq_num > last_seq:
                    last_seq = seq_num
                    print(f"📦 ลำดับ: {seq_num} | พิกัด: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
                else:
                    print(f"⚠️ ข้อมูลหลงลำดับ: {seq_num}")
            else:
                print(f"❌ ขนาดข้อมูลไม่ถูกต้อง: {len(data)} bytes")
                
    except KeyboardInterrupt:
        print("\nหยุดการทำงานฝั่งรับ")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
