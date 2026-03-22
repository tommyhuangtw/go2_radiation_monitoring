#include <Wire.h>

// J305 換算常數：153.8 CPM = 1 uSv/h
#define CONVERSION_FACTOR 153.8
#define LOG_PERIOD 1000 // 每 1 秒輸出一次

volatile unsigned long counts = 0; 
unsigned long previousMillis = 0;

void impulse() {
  counts++;
}

void setup() {
  Serial.begin(9600);
  pinMode(3, INPUT);
  // 監測第 3 號腳位的電壓下降 (FALLING)
  attachInterrupt(digitalPinToInterrupt(3), impulse, FALLING);
  previousMillis = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= LOG_PERIOD) {
    // 拍照存檔並重置計數器
    noInterrupts();
    unsigned long countSnapshot = counts;
    counts = 0;
    interrupts();

    // 計算 CPM 與 uSv/h
    float cpm = countSnapshot * (60000.0 / LOG_PERIOD);
    float uSvh = cpm / CONVERSION_FACTOR;

    // 只輸出純數字 float，方便 Jetson 讀取
    Serial.println(uSvh, 4); 

    previousMillis = currentMillis;
  }
}