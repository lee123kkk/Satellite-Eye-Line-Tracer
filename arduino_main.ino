#include <SoftwareSerial.h>

SoftwareSerial esp32(A1, A0); 

#define ENA 11
#define IN1 7
#define IN2 6
#define IN3 5
#define IN4 4
#define ENB 10

unsigned long lastMsgTime = 0;

void setup() {
  Serial.begin(115200);
  esp32.begin(9600); 
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
}

void controlMotors(float lx, float az) {
  int leftPWM = 0;
  int rightPWM = 0;

  if (abs(az) >= 0.5) {
    // 제자리 회전 모드[cite: 1]
    int turnSpeed = az * 260; 
    leftPWM = turnSpeed;
    rightPWM = -turnSpeed;
  } 
  else if (abs(az) > 0.05) {
    // 부드러운 곡선 주행[cite: 1]
    int baseSpeed = lx * 1000;
    int steer = az * 180; 
    leftPWM = baseSpeed + steer;
    rightPWM = baseSpeed - steer;
  }
  else {
    // 직진[cite: 1]
    int baseSpeed = lx * 1000;
    leftPWM = baseSpeed;
    rightPWM = baseSpeed;
  }

  // 속도 상한선 제한 (안정성 확보)[cite: 1]
  leftPWM = constrain(leftPWM, -160, 150); 
  rightPWM = constrain(rightPWM, -160, 150);

  updateMotors(leftPWM, rightPWM);
}

void updateMotors(int left, int right) {
  digitalWrite(IN1, left >= 0 ? HIGH : LOW);
  digitalWrite(IN2, left >= 0 ? LOW : HIGH);
  analogWrite(ENA, abs(left));
  digitalWrite(IN3, right >= 0 ? HIGH : LOW);
  digitalWrite(IN4, right >= 0 ? LOW : HIGH);
  analogWrite(ENB, abs(right));
}

void loop() {
  if (esp32.available()) {
    String buffer = esp32.readStringUntil('\n');
    if (buffer.startsWith("V,")) {
      int first = buffer.indexOf(',');
      int second = buffer.indexOf(',', first + 1);
      float lx = buffer.substring(first + 1, second).toFloat();
      float az = buffer.substring(second + 1).toFloat();
      controlMotors(lx, az); 
      lastMsgTime = millis();
    }
  }
  if (millis() - lastMsgTime > 500) {
    analogWrite(ENA, 0); analogWrite(ENB, 0);
  }
}