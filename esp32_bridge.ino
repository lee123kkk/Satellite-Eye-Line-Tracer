#include <WiFi.h>

const char* ssid = "asia-edu_2G";
const char* password = "12345678";
const char* host = "192.168.0.125"; // PC IP 주소
const int port = 10000;

WiFiClient client;
unsigned long lastAttempt = 0;

void setup() {
  Serial.begin(115200); // 디버깅용
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // Arduino 통신용[cite: 4, 5]

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }
  Serial.println("WiFi 연결됨");
}

void loop() {
  if (!client.connected()) {
    if (millis() - lastAttempt > 2000) {
      client.connect(host, port);
      lastAttempt = millis();
    }
  }

  if (client.available()) {
    String cmd = client.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      Serial2.println(cmd); // Arduino로 명령 전달[cite: 4]
      Serial.println("To Arduino: " + cmd);
    }
  }
}