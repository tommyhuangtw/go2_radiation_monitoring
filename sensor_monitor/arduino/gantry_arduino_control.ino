/*
  Gantry controller for Arduino Mega + A4988
  龍門 / 升降平台控制程式，適用於 Arduino Mega + A4988

  What this sketch does / 呢個程式做乜：
  - SET_HOME: treat the current physical position as home (0 mm)
    將而家實際位置設成原點 / home（0 mm）
  - GO_HOME: move back to 0 mm
    移動返去原點 0 mm
  - MOVE_TO_TARGET: move to +300 mm from home
    由 home 向上移到 +300 mm
  - STOP: stop immediately
    即時停止
  - STATUS: report current state for Jetson / PC
    回報目前狀態俾 Jetson / 電腦

  Important limitation / 重要限制：
  - With A4988 alone, there is NO real stall detection or sensorless homing.
    只用 A4988 係冇真正 stall detect / 無感歸零功能。
  - "Home" here means: you manually place the gantry at the desired start point,
    then press/set home.
    呢度嘅 home 代表你先人手將平台放到你想要嘅起點，再按 Set Home。
  - For true automatic homing you need a limit switch.
    如果你想真正自動歸零，就要加 limit switch / endstop。

  Serial protocol (115200 baud, newline terminated)
  串口指令格式（115200 baud，每行一個指令）：
    SET_HOME
    GO_HOME
    MOVE_TO_TARGET
    MOVE_ABS <mm>
    MOVE_REL <mm>
    STOP
    STATUS
    ENABLE
    DISABLE
    PING
*/

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------
// Pin assignment / 腳位設定
// ---------------------------
const uint8_t STEP_PIN = 2;  // STEP pulse output / 出 step 脈衝
const uint8_t DIR_PIN  = 3;  // Direction control / 控制方向
const uint8_t EN_PIN   = 4;  // Driver enable (A4988 active LOW) / 驅動器使能（A4988 低電平啟用）

// ---------------------------
// Mechanical configuration / 機械參數
// Adjust these to match your hardware
// 呢幾個數值要跟你實際硬件改
// ---------------------------
const float STEPS_PER_REV = 200.0f;     // 1.8° stepper: 200 full steps per revolution / 1.8 度馬達每圈 200 步
const float MICROSTEPS    = 16.0f;      // Match A4988 MS1/MS2/MS3 setting / 要同 A4988 微步設定一致
const float MM_PER_REV    = 8.0f;       // Leadscrew travel per revolution / 絲桿每轉移動幾多 mm
const float STEPS_PER_MM  = (STEPS_PER_REV * MICROSTEPS) / MM_PER_REV; // Conversion factor / mm 轉步數

// ---------------------------
// Motion settings / 運動設定
// ---------------------------
const float TARGET_HEIGHT_MM = 9.0f;  // 30 cm upward target / 目標高度 30 cm
const float MIN_POS_MM       = 0.0f;    // Soft lower limit / 軟件下限
const float MAX_POS_MM       = 9.0f;  // Soft upper limit / 軟件上限

const unsigned long STEP_INTERVAL_US = 800;  // 1250 steps/sec = 3.125 mm/s / 越細越快
const unsigned long STEP_PULSE_US    = 3;    // A4988 needs a short HIGH pulse / A4988 需要一個短 HIGH 脈衝

// Set this based on your motor wiring and gantry direction.
// 根據你接線同實際上下方向去設定。
// true  -> increasing position means gantry goes up when DIR pin is HIGH.
//         數值增加時，如果 DIR=HIGH 就向上。
// false -> increasing position means gantry goes up when DIR pin is LOW.
//         數值增加時，如果 DIR=LOW 就向上。
const bool DIR_HIGH_IS_UP = true;

// ---------------------------
// State machine / 狀態機
// 用簡單狀態去管理目前動作
// ---------------------------
enum MotionState {
  STATE_IDLE = 0,  // Not moving / 靜止中
  STATE_MOVING,    // Moving / 移動中
  STATE_STOPPED,   // Stopped by command / 被 stop 指令停低
  STATE_ERROR      // Error state / 錯誤狀態
};

MotionState motionState = STATE_IDLE;

bool driverEnabled = true;       // Whether A4988 output is enabled / A4988 有冇啟用輸出
bool homeSet = false;            // Whether user already defined home / 有冇設好 home
bool moveToTargetArmed = false;  // True only for the dedicated +30 cm button / 只喺按 Move +30 cm 時用

long currentStep = 0;            // Current software position in steps / 目前軟件記錄步數
long targetStep = 0;             // Target position in steps / 目標步數
unsigned long lastStepTimeUs = 0; // Last step timestamp / 上一次出 step 時間

char cmdBuffer[64];
uint8_t cmdLen = 0;

// ---------------------------
// Helper functions / 輔助函式
// ---------------------------
float stepsToMm(long steps) {
  // Convert internal step count to mm for display / 將內部步數轉做 mm 方便顯示
  return ((float)steps) / STEPS_PER_MM;
}

long mmToSteps(float mm) {
  // Convert mm command to step count / 將 mm 指令轉成步數
  return lround(mm * STEPS_PER_MM);
}

float currentPosMm() {
  return stepsToMm(currentStep);
}

float targetPosMm() {
  return stepsToMm(targetStep);
}

const char* stateToString(MotionState state) {
  // Convert enum to serial-friendly text / 將狀態 enum 轉做文字，方便俾 Jetson / UI 讀
  switch (state) {
    case STATE_IDLE:    return "IDLE";
    case STATE_MOVING:  return "MOVING";
    case STATE_STOPPED: return "STOPPED";
    case STATE_ERROR:   return "ERROR";
    default:            return "UNKNOWN";
  }
}

void setDriverEnabled(bool enabled) {
  // A4988 EN is active LOW:
  // LOW  = enabled / 開
  // HIGH = disabled / 關
  driverEnabled = enabled;
  digitalWrite(EN_PIN, enabled ? LOW : HIGH);
}

void printStatus() {
  // Send one machine-readable status line to Jetson / PC.
  // 發送一行俾 Jetson / 電腦，格式固定，方便 Python 解析。
  Serial.print("STATUS state=");
  Serial.print(stateToString(motionState));
  Serial.print(" pos_mm=");
  Serial.print(currentPosMm(), 2);
  Serial.print(" target_mm=");
  Serial.print(targetPosMm(), 2);
  Serial.print(" home=");
  Serial.print(homeSet ? 1 : 0);
  Serial.print(" enabled=");
  Serial.println(driverEnabled ? 1 : 0);
}

void announceInfo(const char* text) {
  // Human-readable info message / 一般資訊訊息
  Serial.print("INFO ");
  Serial.println(text);
}

void announceOk(const char* text) {
  // Positive acknowledgement / 成功回覆
  Serial.print("OK ");
  Serial.println(text);
}

void announceErr(const char* text) {
  // Error message / 錯誤回覆
  Serial.print("ERR ");
  Serial.println(text);
}

void stopMotion(const char* reason) {
  // Stop immediately by making target = current position.
  // 透過將 target 設做 current，立即停止之後嘅步進。
  targetStep = currentStep;
  motionState = STATE_STOPPED;
  moveToTargetArmed = false;
  announceOk(reason);
  printStatus();
}

void completeMotion() {
  // Called when we exactly reach targetStep.
  // 當步數到達 targetStep 時會行到呢度。
  motionState = STATE_IDLE;

  // If this motion was triggered by the dedicated "Move +30 cm" button,
  // print the exact message you requested.
  // 如果係按專用 Move +30 cm 按鈕，就輸出你指定嗰句訊息。
  if (moveToTargetArmed && labs(currentStep - mmToSteps(TARGET_HEIGHT_MM)) <= 1) {
    Serial.println("TARGET LOCATION ARRIVED");
  } else {
    announceOk("ARRIVED");
  }

  moveToTargetArmed = false;
  printStatus();
}

bool clampAndValidate(float mm, float &clamped) {
  // Keep motion inside a software safety range.
  // 用軟件限制活動範圍，避免跑出安全範圍。
  if (mm < MIN_POS_MM || mm > MAX_POS_MM) {
    clamped = mm;
    return false;
  }
  clamped = mm;
  return true;
}

void beginMoveToMm(float mm, bool isTargetButton) {
  // Start a move toward an absolute mm target.
  // 以絕對座標 mm 開始移動。

  if (!homeSet) {
    // We only allow motion after user defines where home is.
    // 未 set home 前唔俾郁，避免軟件座標同實際位置唔一致。
    motionState = STATE_ERROR;
    announceErr("HOME_NOT_SET");
    printStatus();
    return;
  }

  float targetMmClamped;
  if (!clampAndValidate(mm, targetMmClamped)) {
    motionState = STATE_ERROR;
    announceErr("OUT_OF_RANGE");
    printStatus();
    return;
  }

  // Auto-enable driver if it was disabled.
  // 如果之前 disable 咗 driver，開始動作前會自動開返。
  if (!driverEnabled) {
    setDriverEnabled(true);
  }

  targetStep = mmToSteps(targetMmClamped);
  moveToTargetArmed = isTargetButton;

  // Already at target -> immediately acknowledge.
  // 如果本身已經喺目標位，就直接回覆到達。
  if (targetStep == currentStep) {
    motionState = STATE_IDLE;
    if (isTargetButton) {
      Serial.println("TARGET LOCATION ARRIVED");
    } else {
      announceOk("ARRIVED");
    }
    printStatus();
    return;
  }

  motionState = STATE_MOVING;
  lastStepTimeUs = micros();
  announceOk(isTargetButton ? "MOVING_TO_TARGET" : "MOVING");
  printStatus();
}

void setDirectionForIncreasingPosition(bool increasing) {
  // Convert logical direction (position increasing / decreasing)
  // into the actual DIR pin level based on your wiring.
  // 將「座標增加 / 減少」轉成實際 DIR 腳位高低電平。
  bool pinLevel;
  if (increasing) {
    pinLevel = DIR_HIGH_IS_UP ? HIGH : LOW;
  } else {
    pinLevel = DIR_HIGH_IS_UP ? LOW : HIGH;
  }
  digitalWrite(DIR_PIN, pinLevel);
}

void doOneStepIfNeeded() {
  // Non-blocking step generator.
  // 非阻塞式出 step，咁 STOP 指令先可以即時插入。
  if (motionState != STATE_MOVING) {
    return;
  }

  unsigned long nowUs = micros();
  if ((unsigned long)(nowUs - lastStepTimeUs) < STEP_INTERVAL_US) {
    return;
  }
  lastStepTimeUs = nowUs;

  bool increasing = (targetStep > currentStep);
  setDirectionForIncreasingPosition(increasing);

  // One physical step pulse.
  // 出一個 step 脈衝。
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(STEP_PIN, LOW);

  // Update software position after the pulse.
  // 出完一步之後更新軟件記錄位置。
  currentStep += increasing ? 1 : -1;

  if (currentStep == targetStep) {
    completeMotion();
  }
}

void uppercase(char *s) {
  // Convert command text to uppercase for case-insensitive matching.
  // 將指令字串轉大楷，令大小寫都接受。
  while (*s) {
    *s = toupper((unsigned char)*s);
    ++s;
  }
}

void trimWhitespace(char *s) {
  // Remove leading / trailing spaces.
  // 刪除前後空白字元。

  // Left trim / 左邊去空白
  char *start = s;
  while (*start && isspace((unsigned char)*start)) {
    ++start;
  }
  if (start != s) {
    memmove(s, start, strlen(start) + 1);
  }

  // Right trim / 右邊去空白
  size_t len = strlen(s);
  while (len > 0 && isspace((unsigned char)s[len - 1])) {
    s[len - 1] = '\0';
    --len;
  }
}

void handleCommand(char *cmd) {
  // Parse one complete serial command line.
  // 解析一整行 serial 指令。
  trimWhitespace(cmd);
  if (strlen(cmd) == 0) {
    return;
  }

  // Keep a copy before uppercasing so numbers remain untouched.
  // 先保留原始字串，等之後讀取數值參數時唔受影響。
  char original[64];
  strncpy(original, cmd, sizeof(original) - 1);
  original[sizeof(original) - 1] = '\0';
  uppercase(cmd);

  if (strcmp(cmd, "PING") == 0) {
    announceOk("PONG");
    printStatus();
    return;
  }

  if (strcmp(cmd, "STATUS") == 0) {
    printStatus();
    return;
  }

  if (strcmp(cmd, "ENABLE") == 0) {
    setDriverEnabled(true);
    announceOk("ENABLED");
    printStatus();
    return;
  }

  if (strcmp(cmd, "DISABLE") == 0) {
    // Stop motion and disable motor holding torque.
    // 停止移動，同時關閉 driver 輸出。
    targetStep = currentStep;
    motionState = STATE_IDLE;
    moveToTargetArmed = false;
    setDriverEnabled(false);
    announceOk("DISABLED");
    printStatus();
    return;
  }

  if (strcmp(cmd, "STOP") == 0) {
    stopMotion("STOPPED");
    return;
  }

  if (strcmp(cmd, "SET_HOME") == 0) {
    // Current physical location becomes 0 mm.
    // 將而家實際位置定義為 0 mm。
    targetStep = currentStep;
    currentStep = 0;
    targetStep = 0;
    homeSet = true;
    motionState = STATE_IDLE;
    moveToTargetArmed = false;
    announceOk("HOME_SET");
    printStatus();
    return;
  }

  if (strcmp(cmd, "GO_HOME") == 0) {
    beginMoveToMm(0.0f, false);
    return;
  }

  if (strcmp(cmd, "MOVE_TO_TARGET") == 0) {
    beginMoveToMm(TARGET_HEIGHT_MM, true);
    return;
  }

  if (strncmp(cmd, "MOVE_ABS", 8) == 0) {
    // Example / 例子: MOVE_ABS 120
    char *spacePtr = strchr(original, ' ');
    if (!spacePtr) {
      motionState = STATE_ERROR;
      announceErr("BAD_MOVE_ABS");
      printStatus();
      return;
    }
    float mm = atof(spacePtr + 1);
    beginMoveToMm(mm, false);
    return;
  }

  if (strncmp(cmd, "MOVE_REL", 8) == 0) {
    // Example / 例子: MOVE_REL -20
    char *spacePtr = strchr(original, ' ');
    if (!spacePtr) {
      motionState = STATE_ERROR;
      announceErr("BAD_MOVE_REL");
      printStatus();
      return;
    }
    float delta = atof(spacePtr + 1);
    beginMoveToMm(currentPosMm() + delta, false);
    return;
  }

  motionState = STATE_ERROR;
  announceErr("UNKNOWN_COMMAND");
  printStatus();
}

void readSerialCommands() {
  // Read bytes from USB serial and assemble full lines.
  // 由 USB serial 逐個 byte 讀入，再拼成完整一行指令。
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuffer[cmdLen] = '\0';
        handleCommand(cmdBuffer);
        cmdLen = 0;
      }
    } else {
      if (cmdLen < sizeof(cmdBuffer) - 1) {
        cmdBuffer[cmdLen++] = c;
      } else {
        // Buffer overflow protection.
        // 防止指令太長導致 buffer overflow。
        cmdLen = 0;
        announceErr("COMMAND_TOO_LONG");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  setDriverEnabled(true);

  delay(300);

  // Boot messages help Jetson know the controller is alive.
  // 開機時輸出提示，方便 Jetson / terminal 知道 Arduino 已經準備好。
  announceInfo("A4988_GANTRY_READY");
  announceInfo("USE_SET_HOME_AT_PHYSICAL_HOME_POSITION");
  printStatus();
}

void loop() {
  // Keep loop short and fast:
  // 1) read serial commands
  // 2) generate step pulses when needed
  // 主循環要保持簡短，咁 stop / status 先會夠即時。
  readSerialCommands();
  doOneStepIfNeeded();
}
