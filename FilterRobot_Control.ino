#include <Servo.h>

// 创建舵机对象
Servo servo1;
Servo servo2;

// 定义引脚
const int SERVO1_PIN = 9;     // 舵机1连接到数字引脚9
const int SERVO2_PIN = 10;    // 舵机2连接到数字引脚10
const int TRIGGER_PIN = 2;    // 触发引脚，连接到数字引脚2

// 变量定义
// 按键判定相关变量
unsigned long pressStartTime = 0;
bool pressActive = false;
bool lastTriggerState = false;
bool isRunning = false; // 任务1运行标志
bool isTask2Running = false; // 任务2运行标志
bool isTask2Returning = false; // 任务2回位标志
unsigned long startTime = 0;
unsigned long task2StartTime = 0;
unsigned long task2ReturnStartTime = 0;
int currentStep = 0;
// 触发标志变量（主循环用）
bool task1Triggered = false;
bool task2Triggered = false;
bool task2ReturnTriggered = false;

// 时间数组，定义每个关键帧的时间点（毫秒）
unsigned long timePoints[] = {0, 5000, 10000, 15000, 20000};
// 舵机运动脉冲宽度数组（微秒）
int servo1PulseWidths[] = {2500, 1420, 1420, 1420, 2500}; // 舵机1: 运行5秒→停留5秒→复位5秒
int servo2PulseWidths[] = {1500, 1500, 2100, 2100, 1500}; // 舵机2: 停留10秒→运行5秒→复位5秒
// 任务2目标位置（可根据实际调整）
const int TASK2_SERVO1_TARGET = 1420;
const int TASK2_SERVO2_TARGET = 2100;
const unsigned long TASK2_DURATION = 3000; // 3秒
const int numSteps = sizeof(servo1PulseWidths) / sizeof(servo1PulseWidths[0]);


// 插值参数
const unsigned long INTERPOLATION_INTERVAL = 20; // 插值更新间隔（毫秒）20ms = 50Hz
float currentServo1PulseWidth = 2500;
float currentServo2PulseWidth = 1500;

void setup() {
  // 初始化串口监视器
  Serial.begin(9600);
  
  // 附加舵机到指定引脚
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  
  // 设置触发引脚为输入模式，启用内部上拉电阻
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  
  // 舵机初始位置
  currentServo1PulseWidth = servo1PulseWidths[0];
  currentServo2PulseWidth = servo2PulseWidths[0];
  servo1.writeMicroseconds((int)currentServo1PulseWidth);
  servo2.writeMicroseconds((int)currentServo2PulseWidth);
  
  Serial.println("舵机控制系统已启动");
  Serial.println("将数字引脚2连接到3.3V来触发舵机运动");
}

void loop() {
  // 读取触发引脚状态（因为使用了上拉电阻，所以需要反转逻辑）
  bool currentTriggerState = !digitalRead(TRIGGER_PIN);

  // 按键按下（上升沿）
  if (currentTriggerState && !lastTriggerState) {
    pressStartTime = millis();
    pressActive = true;
  }
  // 按键持续按压
  if (currentTriggerState && pressActive) {
    unsigned long pressDuration = millis() - pressStartTime;
    // 按下超过1秒，判定为长按
    if (pressDuration >= 1000 && !task2Triggered) {
      task2Triggered = true;
      pressActive = false;
    }
  }
  // 按键松开（下降沿）
  if (!currentTriggerState && lastTriggerState && pressActive) {
    unsigned long pressDuration = millis() - pressStartTime;
    pressActive = false;
    if (pressDuration < 100) {
      Serial.println("误触发短按（按下不足100ms）");
    } else if (pressDuration < 1000) {
      task1Triggered = true;
    }
    // 长按松开，进入回位
    if (task2Triggered) {
      task2Triggered = false;
      task2ReturnTriggered = true;
    }
  }
  lastTriggerState = currentTriggerState;

  // 进入任务1
  if (task1Triggered) {
    runTask1();
    task1Triggered = false;
  }
  // 进入任务2
  if (task2Triggered) {
    runTask2(true); // true表示去目标位
    // 任务2在松开后才回位
  }
  // 进入任务2回位
  if (task2ReturnTriggered) {
    runTask2(false); // false表示回初始位
    task2ReturnTriggered = false;
  }

  delay(10);
}

// 插值函数：根据时间计算当前舵机角度
void interpolateServoPositions(unsigned long elapsedTime) {
  // 找到当前时间所在的时间段
  int segmentIndex = -1;
  for (int i = 0; i < numSteps - 1; i++) {
    if (elapsedTime >= timePoints[i] && elapsedTime < timePoints[i + 1]) {
      segmentIndex = i;
      break;
    }
  }
  
  // 如果超出最后一个时间点，使用最后一段
  if (segmentIndex == -1 && elapsedTime >= timePoints[numSteps - 1]) {
    segmentIndex = numSteps - 2;
  }
  
  if (segmentIndex >= 0) {
    // 计算插值参数 t (0.0 到 1.0)
    unsigned long segmentStart = timePoints[segmentIndex];
    unsigned long segmentEnd = timePoints[segmentIndex + 1];
    unsigned long segmentDuration = segmentEnd - segmentStart;
    unsigned long timeInSegment = elapsedTime - segmentStart;
    
    float t = (float)timeInSegment / (float)segmentDuration;
    
    // 限制 t 在 [0, 1] 范围内
    if (t > 1.0) t = 1.0;
    if (t < 0.0) t = 0.0;
    
    // 使用缓动函数进行平滑插值（ease-in-out）
    t = smoothStep(t);
    
    // 线性插值计算目标脉冲宽度
    float targetServo1 = servo1PulseWidths[segmentIndex] + 
                        t * (servo1PulseWidths[segmentIndex + 1] - servo1PulseWidths[segmentIndex]);
    float targetServo2 = servo2PulseWidths[segmentIndex] + 
                        t * (servo2PulseWidths[segmentIndex + 1] - servo2PulseWidths[segmentIndex]);
    
    // 更新当前脉冲宽度
    currentServo1PulseWidth = targetServo1;
    currentServo2PulseWidth = targetServo2;
    
    // 设置舵机脉冲宽度
    servo1.writeMicroseconds((int)currentServo1PulseWidth);
    servo2.writeMicroseconds((int)currentServo2PulseWidth);
    
    // 输出调试信息（可选，避免过多输出）
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime > 200) { // 每200ms输出一次
      Serial.print("时间: ");
      Serial.print(elapsedTime);
      Serial.print("ms, 舵机1脉宽: ");
      Serial.print(currentServo1PulseWidth, 0);
      Serial.print("μs, 舵机2脉宽: ");
      Serial.print(currentServo2PulseWidth, 0);
      Serial.println("μs");
      lastPrintTime = millis();
    }
  }
}


  void runTask1() {
    Serial.println("任务1开始：运动序列");
    unsigned long startTime = millis();
    while (true) {
      unsigned long currentTime = millis();
      unsigned long elapsedTime = currentTime - startTime;
      if (elapsedTime >= timePoints[numSteps - 1]) {
        Serial.println("任务1完成，舵机回初始位");
        currentServo1PulseWidth = servo1PulseWidths[0];
        currentServo2PulseWidth = servo2PulseWidths[0];
        servo1.writeMicroseconds((int)currentServo1PulseWidth);
        servo2.writeMicroseconds((int)currentServo2PulseWidth);
        break;
      } else {
        interpolateServoPositions(elapsedTime);
      }
      delay(10);
    }
  }

  // 任务2：舵机缓慢转到目标位或回初始位
  void runTask2(bool toTarget) {
    if (toTarget) {
      Serial.println("任务2开始：舵机缓慢转到目标位");
      // 缓慢转到目标位
      unsigned long startTime = millis();
      while (true) {
        unsigned long elapsed = millis() - startTime;
        float t = (float)elapsed / (float)TASK2_DURATION;
        if (t > 1.0) t = 1.0;
        t = smoothStep(t);
        float targetServo1 = servo1PulseWidths[0] + t * (TASK2_SERVO1_TARGET - servo1PulseWidths[0]);
        float targetServo2 = servo2PulseWidths[0] + t * (TASK2_SERVO2_TARGET - servo2PulseWidths[0]);
        currentServo1PulseWidth = targetServo1;
        currentServo2PulseWidth = targetServo2;
        servo1.writeMicroseconds((int)currentServo1PulseWidth);
        servo2.writeMicroseconds((int)currentServo2PulseWidth);
        // 可选调试输出
        static unsigned long lastPrintTime = 0;
        if (millis() - lastPrintTime > 200) {
          Serial.print("任务2进行中，舵机1脉宽: ");
          Serial.print(currentServo1PulseWidth, 0);
          Serial.print("μs, 舵机2脉宽: ");
          Serial.print(currentServo2PulseWidth, 0);
          Serial.println("μs");
          lastPrintTime = millis();
        }
        if (t >= 1.0) {
          break;
        }
        delay(10);
      }
      // 保持目标位，持续检测按键状态
      while (true) {
        // 按键是否松开
        bool keyState = !digitalRead(TRIGGER_PIN);
        if (!keyState) {
          Serial.println("任务2检测到松开，舵机缓慢回初始位");
          break;
        }
        // 按下时保持目标位
        servo1.writeMicroseconds((int)TASK2_SERVO1_TARGET);
        servo2.writeMicroseconds((int)TASK2_SERVO2_TARGET);
        delay(10);
      }
      // 松开后缓慢回初始位，回位期间不再检测按键
      unsigned long returnStart = millis();
      while (true) {
        unsigned long returnElapsed = millis() - returnStart;
        float t = (float)returnElapsed / (float)TASK2_DURATION;
        if (t > 1.0) t = 1.0;
        t = smoothStep(t);
        // 从目标位平滑回到初始位
        float targetServo1 = TASK2_SERVO1_TARGET + t * (servo1PulseWidths[0] - TASK2_SERVO1_TARGET);
        float targetServo2 = TASK2_SERVO2_TARGET + t * (servo2PulseWidths[0] - TASK2_SERVO2_TARGET);
        currentServo1PulseWidth = targetServo1;
        currentServo2PulseWidth = targetServo2;
        servo1.writeMicroseconds((int)currentServo1PulseWidth);
        servo2.writeMicroseconds((int)currentServo2PulseWidth);
        // 可选调试输出
        static unsigned long lastPrintTime2 = 0;
        if (millis() - lastPrintTime2 > 200) {
          Serial.print("任务2回位中，舵机1脉宽: ");
          Serial.print(currentServo1PulseWidth, 0);
          Serial.print("μs, 舵机2脉宽: ");
          Serial.print(currentServo2PulseWidth, 0);
          Serial.println("μs");
          lastPrintTime2 = millis();
        }
        if (t >= 1.0) {
          Serial.println("任务2回位完成，等待下一次触发。");
          // 清除所有相关触发标志，确保系统回到主循环
          task2Triggered = false;
          task2ReturnTriggered = false;
          break;
        }
        delay(10);
      }
    }
  }
// 平滑步进函数 (Smooth Step)，提供缓动效果
float smoothStep(float t) {
  // 3t² - 2t³ 公式提供平滑的缓入缓出效果
  return t * t * (3.0 - 2.0 * t);
}
