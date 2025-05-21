#include <SPI.h>
#include <mcp2515.h>

#define INT_PIN 4  // ขา INT จาก MCP2515 ต่อกับ GPIO4

MCP2515 mcp2515(5);  // CS = GPIO5
struct can_frame canMsg;
volatile bool can_interrupt = false;

void IRAM_ATTR onCANInterrupt() {
  can_interrupt = true;
}

void setup() {
  Serial.begin(115200);
  SPI.begin(18, 19, 23, 5); // SCK, MISO, MOSI, SS

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);  // ปรับตามความถี่คริสตัลบนบอร์ด MCP2515
  mcp2515.setNormalMode();

  pinMode(INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), onCANInterrupt, FALLING);

  Serial.println("✅ MCP2515 Ready, กำลังส่งคำสั่งไปยัง ZLAC...");

  // ส่งคำสั่ง SDO Write → ZLAC
  sendCAN(0x601, {0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00}); // Set Mode CSV
  sendCAN(0x601, {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00}); // Set Mode CSV
  sendCAN(0x601, {0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00}); // Set Mode CSV
  sendCAN(0x601, {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00}); // Set Mode CSV
  sendCAN(0x601, {0x23, 0xFF, 0x60, 0x03, 0x64, 0x00, 0x64, 0x00}); // Set Mode CSV

  delay(5000);
  sendCAN(0x601, {0x23, 0xFF, 0x60, 0x03, 0x00, 0x00, 0x00, 0x00}); // Set Mode CSV
  sendCAN(0x601, {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00}); // Set Mode CSV
  //sendCAN(0x601, {0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00}); // Set Mode CSV
  //sendCAN(0x601, {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00}); // Set Mode CSV
}

void loop() {
  if (can_interrupt) {
    can_interrupt = false;

    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
      Serial.print("📥 รับจาก ID: 0x");
      Serial.print(canMsg.can_id, HEX);
      Serial.print(" → ");

      for (int i = 0; i < canMsg.can_dlc; i++) {
        Serial.print("0x");
        Serial.print(canMsg.data[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      if (canMsg.can_id == 0x581) {
        Serial.println("✅ ได้รับการตอบกลับจาก ZLAC (SDO Response)");
      }
    }
  }

  delay(1);  // ปล่อย CPU บ้าง
}

void sendCAN(uint16_t id, std::initializer_list<uint8_t> data) {
  struct can_frame msg;
  msg.can_id = id;
  msg.can_dlc = data.size();

  int i = 0;
  for (auto b : data) msg.data[i++] = b;

  if (mcp2515.sendMessage(&msg) == MCP2515::ERROR_OK) {
    Serial.print("🚀 ส่ง: 0x");
    Serial.println(id, HEX);
  } else {
    Serial.println("❌ ส่งไม่สำเร็จ");
  }

  delay(100); // รอการตอบกลับ
}