/*
 * Sound Sensor Test for Go2 Patrol System
 * Reads Sound Sensor AO and prints value to Serial.
 */

#define SOUND_PIN A0  // 接在類比引腳 A0

void setup() {
  Serial.begin(9600);
  pinMode(SOUND_PIN, INPUT);
  Serial.println("Sound Sensor Test Initialized");
}

void loop() {
  // 讀取聲音強度 (0-1023)
  int soundLevel = analogRead(SOUND_PIN);

  // 印出數值
  Serial.print("Sound Intensity: ");
  Serial.println(soundLevel);

  // 稍微延遲讓數據比較容易閱讀
  delay(300);
}
