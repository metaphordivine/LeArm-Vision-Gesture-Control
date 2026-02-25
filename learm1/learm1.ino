#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUdp.h>
#include "Hiwonder.hpp"
#include "Robot_arm.hpp"

// ================= 1. 基础配置 =================
LeArm_t arm;
WebServer server(80);
WiFiUDP udp;
const int udpPort = 8888; // UDP 监听端口
const char* ssid = "jaTest";
const char* password = "jatestest";

// ================= 2. 核心控制函数 =================
// 增加软限位保护，防止舵机烧毁 (参数为角度)
float clampAngle(float angle, float min_ang, float max_ang) {
  if(angle < min_ang) return min_ang;
  if(angle > max_ang) return max_ang;
  return angle;
}

struct UdpPacket {
  uint8_t head1;     // 0xAA
  uint8_t head2;     // 0x55
  uint8_t mode;      // 0=空闲, 1=跟随, 2=抓取
  uint8_t reserved;
  float x;
  float y;
  float z;
  float c;
};

// ===================== 智能 IK 包装函数（推荐） =====================
void moveToPos(float x, float y, float z, float claw_val, uint32_t moveTime)
{
    float ox = x, oy = y, oz = z;

    // ---- 1. 基础安全限制 ----
    x = clampAngle(x, 8.0, 25.0);     // 前后范围
    y = clampAngle(y, -13.0, 13.0);   // 左右范围
    z = clampAngle(z, 2.0, 25.0);     // 高度范围
    claw_val = clampAngle(claw_val, 10.0, 90.0);

    // ---- 2. 根据 Z 自动调整末端 pitch ----
    // Z=5cm → pitch=-90°（垂直向下）
    // Z=18cm → pitch=0°（水平）
    float pitch = -90.0 + (z - 5.0) * (90.0 / 13.0);
    pitch = clampAngle(pitch, -90.0, 0.0);

    // ---- 3. 自动生成 IK 搜索范围 ----
    float min_pitch = pitch - 60.0;
    float max_pitch = pitch + 60.0;

    min_pitch = clampAngle(min_pitch, -120.0, 40.0);
    max_pitch = clampAngle(max_pitch, -120.0, 40.0);

    // ---- 4. 调用 IK ----
    bool ok = arm.coordinate_set(x, y, z, pitch, min_pitch, max_pitch, moveTime);

    if (!ok) {
        Serial.printf("IK失败: X:%.1f Y:%.1f Z:%.1f pitch=%.1f\n",
                      ox, oy, oz, pitch);
        return;
    }

    // ---- 5. IK 成功后再控制夹爪 ----
    arm.claw_set(claw_val, moveTime);
}


// --- HTTP 接口 (用于视觉抓取) ---
void handleRobot() {
  uint32_t moveTime = server.hasArg("t") ? server.arg("t").toInt() : 500;
  if (server.hasArg("x") && server.hasArg("y")) {
    float x = server.arg("x").toFloat();
    float y = server.arg("y").toFloat();
    float z = server.hasArg("z") ? server.arg("z").toFloat() : 5.0;
    float c = server.hasArg("c") ? server.arg("c").toFloat() : 90.0;
    
    moveToPos(x, y, z, c, moveTime);
    Serial.printf("HTTP IK -> X:%.1f Y:%.1f Z:%.1f C:%.1f\n", x, y, z, c);
  }
  server.send(200, "text/plain", "OK");
}

// ================= 3. 初始化与主循环 =================
void setup() {
  Serial.begin(115200);
  arm.init();
  uint32_t startupTime = 2000; // 设置为 2 秒，让机械臂平稳、缓慢地立起来，防止晃动
  
  arm.knot_run(6, 1500, startupTime); // 底座：转向正前方
  arm.knot_run(5, 1500, startupTime); // 大臂：竖直
  arm.knot_run(4, 1500, startupTime); // 小臂：竖直
  arm.knot_run(3, 1500, startupTime); // 腕部俯仰：竖直
  arm.knot_run(2, 1500, startupTime); // 腕部旋转：中位
  arm.knot_run(1, 1500, startupTime); // 夹爪：半开状态

  pinMode(IO_LED, OUTPUT);
  digitalWrite(IO_LED, LOW);

  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) { 
    delay(500); 
    Serial.print("."); 
  }

  digitalWrite(IO_LED, HIGH);
  Serial.println("\nWiFi Connected!");
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  server.on("/robot", HTTP_GET, handleRobot);
  server.begin();
  
  udp.begin(udpPort); // 启动 UDP 监听
  Serial.println("UDP Server started at port 8888");
}

void loop() {
  server.handleClient();

  int packetSize = udp.parsePacket();
  if (packetSize != sizeof(UdpPacket)) {
    udp.flush();  // 丢弃异常数据包
    return;
  }
  if (packetSize == sizeof(UdpPacket)) {

    UdpPacket pkt;
    udp.read((uint8_t*)&pkt, sizeof(pkt));

    if (pkt.head1 == 0xAA && pkt.head2 == 0x55) {

      switch(pkt.mode) {

        case 1: // 手势跟随
          moveToPos(pkt.x, pkt.y, pkt.z, pkt.c, 50);
          break;

        case 2: // 视觉抓取（如果你以后需要）
          moveToPos(pkt.x, pkt.y, pkt.z, pkt.c, 300);
          break;

        default:
          break;
      }
    }
  }
}
