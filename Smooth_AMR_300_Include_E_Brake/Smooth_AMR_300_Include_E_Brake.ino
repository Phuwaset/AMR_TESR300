#include <SPI.h>
#include <mcp2515.h>

#define CH2 13  // เดินหน้า-ถอยหลัง
#define CH3 14  // ปรับความเร็ว
#define CH4 15  // ซ้าย-ขวา (แทน omega rad/s)
#define CH7 12  // สลับโหมด (0 = Serial2, 1 = RC)
#define AIRBRAKE_PIN 27  // 🛑 ปุ่มหยุดฉุกเฉิน (Airbrake)

struct can_frame canMsg;
MCP2515 mcp2515(5);  // CS = D5

float baseSpeed_mps = 0.8;
const float MPS_TO_RPM = 136.4;
const float wheelDistance = 0.332;

float x = 0, z = 0;
float target_x = 0, target_z = 0;

int prevMode = -1;

void setup() {
  delay(500);
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH7, INPUT);
  pinMode(AIRBRAKE_PIN, INPUT_PULLDOWN);  // กดติด (ต่อกับ GND)

  enableMotor();
  Serial.println("🚗 ระบบพร้อม (CH7 เลือกโหมด RC หรือ Serial2)");
}

void loop() {
  // 🛑 ตรวจสอบ Airbrake ก่อนทำอย่างอื่น
  if (digitalRead(AIRBRAKE_PIN) == HIGH) {
    Serial.println("🛑 เข้าสู่โหมด AIRBRAKE → หยุดมอเตอร์");

    struct can_frame msg;
    msg.can_id = 0x601;
    msg.can_dlc = 8;
    msg.data[0] = 0x2B;
    msg.data[1] = 0x40;
    msg.data[2] = 0x60;
    msg.data[3] = 0x00;
    msg.data[4] = 0x06;
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;

    while (digitalRead(AIRBRAKE_PIN) == HIGH) {
      mcp2515.sendMessage(&msg);
      delay(200);
    }
    enableMotor();
    Serial.println("✅ ออกจากโหมด AIRBRAKE → กลับสู่ระบบปกติ");
    delay(500);
    return;  // ข้ามการทำงานอื่นใน loop นี้
  }

  // 🧭 เลือกโหมดควบคุม
  int ch7_us = pulseIn(CH7, HIGH, 25000);
  int ch7 = (ch7_us >= 1500) ? 1 : 0;

  if (ch7 != prevMode) {
    if (ch7 == 0) {
      target_x = 0;
      target_z = 0;
      Serial.println("🔁 สลับไปโหมด RC → Reset x/z");
    } else {
      Serial.println("🔁 สลับไปโหมด Serial2 → รอรับคำสั่ง");
    }
    prevMode = ch7;
  }

  if (ch7 == 1) {
    if (Serial2.available()) {
      String received = Serial2.readStringUntil('\n');
      received.trim();
      int commaIndex = received.indexOf(',');
      if (commaIndex != -1) {
        String xStr = received.substring(0, commaIndex);
        String zStr = received.substring(commaIndex + 1);
        target_x = xStr.toFloat();
        target_z = zStr.toFloat();

        Serial.print("🔷 จาก Serial2 → X = ");
        Serial.print(target_x, 3);
        Serial.print(" Z = ");
        Serial.println(target_z, 3);
      } else {
        Serial.println("⚠️ Serial2: รูปแบบข้อมูลผิด");
      }
    }
  } else {
    int ch2 = map(constrain(pulseIn(CH2, HIGH, 50000), 500, 2500), 500, 2500, 0, 100);
    int ch3 = map(constrain(pulseIn(CH3, HIGH, 50000), 1000, 2000), 1000, 2000, -100, 100);
    int ch4 = map(constrain(pulseIn(CH4, HIGH, 50000), 500, 2500), 500, 2500, -100, 100);

    float speedFactor = ch2 / 100.0;
    float forward = (ch3 / 100.0) * speedFactor * baseSpeed_mps;
    float maxOmega = 0.8;
    float omega = 0;

    if (abs(ch3) < 5 && abs(ch4) > 5) {
      omega = -(ch4 / 100.0) * maxOmega * speedFactor;
    } else {
      omega = -(ch4 / 100.0) * maxOmega * speedFactor * ((forward >= 0) ? 1 : -1);
    }

    target_x = forward;
    target_z = omega;

    Serial.print("🕹️ จาก RC → X = ");
    Serial.print(target_x, 3);
    Serial.print(" Z = ");
    Serial.println(target_z, 3);
  }

  float step = 0.05;
  x = smoothUpdate(x, target_x, step);
  z = smoothUpdate(z, target_z, step);

  float left_mps = x - (z * wheelDistance / 2.0);
  float right_mps = x + (z * wheelDistance / 2.0);
  setSpeed(left_mps, right_mps);

  Serial.print("🚀 L = ");
  Serial.print(left_mps, 2);
  Serial.print(" R = ");
  Serial.println(right_mps, 2);

  delay(50);
}

float smoothUpdate(float current, float target, float step) {
  if (abs(target - current) <= step) return target;
  return current + ((target > current) ? step : -step);
}

void setSpeed(float left_mps, float right_mps) {
  int16_t left_rpm = left_mps * MPS_TO_RPM;
  int16_t right_rpm = -right_mps * MPS_TO_RPM;

  canMsg.can_id  = 0x601;
  canMsg.can_dlc = 8;
  canMsg.data[0] = 0x23;
  canMsg.data[1] = 0xFF;
  canMsg.data[2] = 0x60;
  canMsg.data[3] = 0x03;
  canMsg.data[4] = left_rpm & 0xFF;
  canMsg.data[5] = (left_rpm >> 8) & 0xFF;
  canMsg.data[6] = right_rpm & 0xFF;
  canMsg.data[7] = (right_rpm >> 8) & 0xFF;

  mcp2515.sendMessage(&canMsg);
}

void enableMotor() {
  struct can_frame msg;
  uint32_t enables[3] = {0x000006, 0x000007, 0x00000F};

  for (int repeat = 0; repeat < 3; repeat++) {
    for (int i = 0; i < 3; i++) {
      msg.can_id = 0x601;
      msg.can_dlc = 8;
      msg.data[0] = 0x2B;
      msg.data[1] = 0x40;
      msg.data[2] = 0x60;
      msg.data[3] = 0x00;
      msg.data[4] = enables[i] & 0xFF;
      msg.data[5] = (enables[i] >> 8) & 0xFF;
      msg.data[6] = 0x00;
      msg.data[7] = 0x00;

      mcp2515.sendMessage(&msg);
      delay(10);
    }
  }

  Serial.println("✅ Enable Motor ผ่าน CAN แล้ว");
}
