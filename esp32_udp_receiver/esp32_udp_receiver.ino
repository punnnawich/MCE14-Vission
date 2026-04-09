#include <WiFi.h>
#include <WiFiUdp.h>

// ==========================================
// การตั้งค่า WiFi
// ==========================================
const char* ssid = "MCE14";      
const char* password = "12345678"; 
const int localUdpPort = 12345; 

WiFiUDP udp;

// ตัวแปรสำหรับเช็คข้อมูลที่มาหลงลำดับ (Out-of-order)
uint32_t lastSeqNum = 0;
bool isFirstPacket = true;

// ตัวแปรสำหรับเช็คสถานะ WiFi
unsigned long lastWiFiCheck = 0;

void setupWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  // ป้องกันการค้างลูปถาวร กรณีเราเตอร์ล่ม ให้รอแค่ 10 วินาทีต่อรอบ
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP()); 
    udp.begin(localUdpPort);
  } else {
    Serial.println("\nWiFi Failed to connect. Will retry later.");
  }
}

void setup() {
  Serial.begin(115200);
  setupWiFi();
}

void loop() {
  // 1. ระบบรักษาการเชื่อมต่อ (Auto Reconnect)
  // หากหลุด WiFi จะพยายามต่อใหม่ทุก 5 วินาที โดยไม่ทำให้โค้ดส่วนอื่นค้างตลอดไป
  if (millis() - lastWiFiCheck > 5000) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi connection lost. Reconnecting...");
      setupWiFi();
    }
    lastWiFiCheck = millis();
  }

  // 2. ตรวจสอบข้อมูล UDP
  int packetSize = udp.parsePacket();
  if (packetSize) {
    
    // คาดหวังแพ็กเกจจนาด 16 Bytes (Sequence 4 + X 4 + Y 4 + Z 4)
    if (packetSize == 16) {
      uint32_t seqNum;
      float x, y, z;
      
      // อ่านข้อมูลทีละ 4 bytes ลงในตัวแปร
      udp.read((char*)&seqNum, 4);
      udp.read((char*)&x, 4);
      udp.read((char*)&y, 4);
      udp.read((char*)&z, 4);
      
      // 3. ป้องกันข้อมูลหลงลำดับ (Out-of-Order Packets Guard)
      // บางครั้ง UDP แพ็กเกจใหม่เดินทางมาถึงก่อนหน้าแพ็กเกจเก่า 
      // ถ้ารับแพ็กเกจเก่าไปใช้ หุ่นยนต์อาจจะกระตุกถอยหลังชั่วขณะ
      if (isFirstPacket || seqNum > lastSeqNum) {
        lastSeqNum = seqNum;
        isFirstPacket = false;
        
        // Print ดูค่า (เอาออกได้เพื่อให้ทำงานไวขึ้นสุดขีด)
        Serial.print("Seq:"); Serial.print(seqNum);
        Serial.print("\tX:"); Serial.print(x, 2);
        Serial.print(" Y:"); Serial.print(y, 2);
        Serial.print(" Z:"); Serial.println(z, 2);

        // --- เพิ่มโค้ดควบคุม Motor / Servo ตรงนี้ ---
        
      } else {
        // หากมีแพ็กเกจที่ช้าและเก่ากว่าค่าล่าสุดมาถึง ให้ทิ้งไป
        // Serial.println("Ignored old packet");
      }
      
    } else {
      // ล้างข้อมูลขยะที่ส่งมาผิดขนาดออกจาก Buffer
      udp.flush();
    }
  }
  
  // ระวัง: ห้ามใช้ delay() นานๆ ใน loop เพื่อให้รอบการรันเร็วที่สุด
}
