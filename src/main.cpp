#include <Arduino.h> 
#include <math.h>
//input
float GLOBAL_TARGET_ANGLE = 300; // 目標角度 (360度)
float totalAngleError;

//PID 參數
int Base_power = 57;                // 基礎輸出功率
float leftPower = 0;   
float rightPower = 0;  
float current_rightRPM = 0;   // 實際右輪轉速
float current_leftRPM = 0;    // 實際左輪轉速  
float rintegralError = 0;      // 右累積誤差
float lintegralError = 0;      // 左累積誤差
float rLastError = 0;      // 右前次誤差
float lLastError = 0;      // 左前次誤差
// PID 參數結構
struct PIDParams {
    float kp;
    float ki;
    float kd;
    float targetRPM;          // 目標轉速
    float pError;          // P 誤差
    float iError;          // 積分誤差
    float dError;          // 微分誤差
};

// PID陣列：0左輪, 1右輪
PIDParams anglePid[2] = {
    {0.25, 0.018, 0.24, 0, 0, 0},  // 左輪 (lKp, lKi, lKd, lTargetRPM, lintegralError, lLastError)
    {0.33, 0.018, 0.24, 0, 0, 0}  // 右輪 (rKp, rKi, rKd, rTargetRPM, rintegralError, rLastError)
};

//右輪
const int ENA =  5;  //速度控制
const int IN1 = 13;  //方向控制 1
const int IN2 = 12;  //方向控制 2
const int rencoderPinA = 2; 
const int rencoderPinB = 8;
volatile long rCount = 0;   // 右輪總脈衝
long lastRCount = 0;        // 上一次紀錄的脈衝
unsigned long prevTime = 0; // 上一次計算的時間
float rightRPM = 0;         // 右輪轉速
const int r_rotation = 1;   // 右輪旋轉方向 (1 向前、正轉；-1 向後、反轉)
//左輪
const int ENB = 6;   //速度控制
const int IN3 = 11;  //方向控制 1
const int IN4 = 10;  //方向控制 2
const int lencoderPinA = 3;  
const int lencoderPinB = 7;
volatile long lCount = 0;   // 左輪總脈衝
long lastlCount = 0;        // 上一次紀錄的脈衝
float leftRPM = 0;          // 左輪轉速
const int l_rotation = 1;   // 左輪旋轉方向 (1 向前、正轉；-1 向後、反轉)

// 馬達參數
const int GEAR_RATIO = 30;  
const int PPR = 11;         
const float TOTAL_PPR = (float)PPR *GEAR_RATIO;

//車體參數
float wheel_perimeter = 0.069*M_PI;  // 車輪周長 (直徑*3.14159) (m)
float wheel_base = 0.1975;           // 車輪距離 (m)
float milage[2] = {0.0, 0.0};        // 里程 (m)[左輪, 右輪]

void doRencoder() {
  if (digitalRead(rencoderPinA) == digitalRead(rencoderPinB)) { // 比較 A, B 相
    rCount++;
  } else {
    rCount--;
  }
}

void dolencoder() {
  if (digitalRead(lencoderPinA) == digitalRead(lencoderPinB)) { // 比較 A, B 相
    lCount++;
  } else {
    lCount--;
  }
}

void setMotorDirection(float rTarget, float lTarget) {
  // 右輪
  digitalWrite(IN1, (rTarget >= 0) ? LOW : HIGH);
  digitalWrite(IN2, (rTarget >= 0) ? HIGH : LOW);
  
  // 左輪
  digitalWrite(IN3, (lTarget >= 0) ? LOW : HIGH);
  digitalWrite(IN4, (lTarget >= 0) ? HIGH : LOW);
}

float turnedAngle(float milage[2],float wheel_base) {
  // 轉過的角度 = acos(1-0.5*pow((里程差), 2)/pow(車輪距離, 2)) * (180.0 / 3.14159)
  double turned_angle = 0;
  if (milage[0] > milage[1]) {
    turned_angle = acos(1-0.5*pow((milage[0]-milage[1]), 2)/pow(wheel_base, 2))*180/M_PI; // 如果左輪里程大於右輪里程，則將左輪里程設為0，避免計算出負角度
  }
  else {
    turned_angle = acos(1-0.5*pow((milage[1]-milage[0]), 2)/pow(wheel_base, 2))*180/M_PI; // 如果右輪里程大於左輪里程，則將右輪里程設為0，避免計算出負角度
  }

  return turned_angle;
}

void setup() {
  rCount=0;
  lCount=0;
  //右輪
  pinMode(rencoderPinA, INPUT_PULLUP);
  pinMode(rencoderPinB, INPUT_PULLUP);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  //左輪
  pinMode(lencoderPinA, INPUT_PULLUP);
  pinMode(lencoderPinB, INPUT_PULLUP);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(rencoderPinA), doRencoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(lencoderPinA), dolencoder, CHANGE);
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - prevTime >= 100) {
    noInterrupts();
    long currentRCount = rCount;    long currentlCount = lCount;
    interrupts();
    
    // 1. 計算兩輪的實體里程 (保留正負號)
    milage[0] = (currentlCount / (TOTAL_PPR * 2.0)) * wheel_perimeter;
    milage[1] = (currentRCount / (TOTAL_PPR * 2.0)) * wheel_perimeter;

    // 2. 算出車體目前的「絕對航向角度」
    float currentHeading = ((milage[0] - milage[1]) / wheel_base) * (180.0 / M_PI);

    // 3. 將當前航向標準化在 0 ~ 360 度空間
    while (currentHeading < 0.0)   currentHeading += 360.0;
    while (currentHeading >= 360.0) currentHeading -= 360.0;

    // 4. 計算總角度誤差（絕對不修改全域變數 GLOBAL_TARGET_ANGLE）
    float totalAngleError = GLOBAL_TARGET_ANGLE - currentHeading;

    // 5. 【動態追隨核心】自動換算為 -180 ~ +180 之間的最短路徑
    // 這樣可以保證：
    // - 當目標在 0~180 時自動左轉，在 180~360 時自動右轉
    // - 遇到 359度 跳到 1度 這種臨界點時，會絲滑地順向轉過去，絕對不會爆轉大甩頭
    if (totalAngleError > 180.0)  totalAngleError -= 360.0;
    if (totalAngleError < -180.0) totalAngleError += 360.0;

    Serial.print("Ext_Target: "); Serial.print(GLOBAL_TARGET_ANGLE);
    Serial.print(" | Current: "); Serial.print(currentHeading);
    Serial.print(" | Error: "); Serial.println(totalAngleError);

    float min_ratio = 0.43;         // 最低保留 25% 的控制力道，避免低速卡死
    float start_decel_angle = 100.0; // 剩餘 50 度以內開始平滑減速
    
    // 計算歸一化誤差比例
    float x = abs(totalAngleError) / start_decel_angle;
    if (x > 1.0) x = 1.0; // 超過 50 度就維持 1.0 原速

    // 你的經典餘弦平滑數學式
    float velocity_factor = min_ratio + (1.0 - min_ratio) * ((1.0 - cos(x * M_PI)) / 2.0);

    // 6. PID 核心計算：直接用有正負號的 totalAngleError
    // --- 右輪 PID ---
    anglePid[1].pError = totalAngleError; 
    anglePid[1].iError += anglePid[1].pError;
    anglePid[1].iError = constrain(anglePid[1].iError, -150, 150); // 限制積分防止暴衝
    anglePid[1].dError = anglePid[1].pError - rLastError;
    rLastError = anglePid[1].pError;

    // --- 左輪 PID ---
    anglePid[0].pError = -totalAngleError; // 與右輪反向，實現原地旋轉/追隨
    anglePid[0].iError += anglePid[0].pError;
    anglePid[0].iError = constrain(anglePid[0].iError, -150, 150);
    anglePid[0].dError = anglePid[0].pError - lLastError;
    lLastError = anglePid[0].pError;

    
    // 7. 計算馬達功率輸出 (提醒：外部動態輸入時，kp 建議先從 0.1 ~ 0.3 開始抓感覺)
    rightPower = ((anglePid[1].pError * anglePid[1].kp) + (anglePid[1].iError * anglePid[1].ki) + (anglePid[1].dError * anglePid[1].kd)) * velocity_factor;
    leftPower  = ((anglePid[0].pError * anglePid[0].kp) + (anglePid[0].iError * anglePid[0].ki) + (anglePid[0].dError * anglePid[0].kd)) * velocity_factor;

    // 8. 加入死區基底功率，幫助低溫、低電量時克服齒輪箱摩擦力
    if (rightPower > 1.0)       rightPower += Base_power;
    else if (rightPower < -1.0) rightPower -= Base_power;
    else rightPower = 0;

    if (leftPower > 1.0)       leftPower += (Base_power + 20);
    else if (leftPower < -1.0) leftPower -= (Base_power + 20);
    else leftPower = 0;

    // 9. 微幅死區平滑控制：當外部輸入非常接近當前角度（誤差小於 1.5 度）
    // 直接清空積分與輸出，防止馬達在目標點附近發生連續訊號的微小高頻抖動
    if (abs(totalAngleError) <= 1.5) {
        rightPower = 0; leftPower = 0;
        anglePid[1].iError = 0; anglePid[0].iError = 0;
    }

    // 10. 實體驅動：動態調整正反轉腳位
    // 右輪
    digitalWrite(IN1, (rightPower >= 0) ? LOW : HIGH);
    digitalWrite(IN2, (rightPower >= 0) ? HIGH : LOW);
    analogWrite(ENA, constrain(abs((int)rightPower), 0, 255));

    // 左輪
    digitalWrite(IN3, (leftPower >= 0) ? LOW : HIGH);
    digitalWrite(IN4, (leftPower >= 0) ? HIGH : LOW);
    analogWrite(ENB, constrain(abs((int)leftPower), 0, 255));
    
    prevTime = currentTime;
  }
}
