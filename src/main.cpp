#include <Arduino.h> 
#include <math.h>
//input
float targetangle = 80; // 目標角度 (360度)

//PID 參數
int Base_power = 52;                // 基礎輸出功率
float Base_RPM = 25;                // 基礎轉速
float Base_linerVelocity = 0.1;     // 線速度 (m/s)
float rtargetRPM = 50;   // 右目標轉速
float ltargetRPM = 0;    // 左目標轉速
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
PIDParams pid[2] = {
    {0.4, 0.55, 0.5, 0, 0, 0},  // 左輪 (lKp, lKi, lKd, lTargetRPM, lintegralError, lLastError)
    {0.4, 0.55, 0.6, 0, 0, 0}  // 右輪 (rKp, rKi, rKd, rTargetRPM, rintegralError, rLastError)
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

  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(rencoderPinA), doRencoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(lencoderPinA), dolencoder, CHANGE);
}

void setMotorDirection(float rTarget, float lTarget) {
  // 右輪
  digitalWrite(IN1, (rTarget >= 0) ? LOW : HIGH);
  digitalWrite(IN2, (rTarget >= 0) ? HIGH : LOW);
  
  // 左輪
  digitalWrite(IN3, (lTarget >= 0) ? LOW : HIGH);
  digitalWrite(IN4, (lTarget >= 0) ? HIGH : LOW);
}

void angleTotargetRPM(float linearVelocity, float base_RPM, float targetAngle) {
  float vRight, vLeft;

  float turnOffset = targetAngle * 0.1;


  if (targetAngle > 2.0 && targetAngle <= 179.0) {
    // 0-180 度：左轉
    vRight = linearVelocity + (base_RPM/60*wheel_perimeter);
    vLeft  = linearVelocity - (base_RPM/60*wheel_perimeter);
  } 
  else if (targetAngle < 358 && targetAngle >= 181) {
    // 180-360 度：右轉
    vRight = linearVelocity - (base_RPM/60*wheel_perimeter);
    vLeft  = linearVelocity + (base_RPM/60*wheel_perimeter);
  }

  // v to rpm

  pid[1].targetRPM = (vRight / wheel_perimeter) * 60.0 + turnOffset;
  pid[0].targetRPM  = (vLeft / wheel_perimeter) * 60.0 - turnOffset;
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

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - prevTime >= 100) {
    Serial.print(currentTime);
    // 1.100ms 內的脈衝差
    noInterrupts(); // 讀取時暫时關閉中斷防止數據出錯
    long currentRCount = rCount;    long currentlCount = lCount;
    interrupts();
    
    //里程
    milage[0] = (currentlCount / (TOTAL_PPR * 2.0)) * wheel_perimeter;
    milage[1] = (currentRCount / (TOTAL_PPR * 2.0)) * wheel_perimeter;

    float remainingAngle;
    if (targetangle > 0) {
      remainingAngle = targetangle - turnedAngle(milage, wheel_base); 
    }
    else {
      remainingAngle = targetangle + turnedAngle(milage, wheel_base); 
    }

    Serial.print("remainingAngle:");
    Serial.println(remainingAngle);

    if (remainingAngle > 2.0 && remainingAngle <= 179.0 || remainingAngle < 358 && remainingAngle >= 181) {
        angleTotargetRPM(Base_linerVelocity, Base_RPM, remainingAngle);
    } else {
        pid[0].targetRPM = 0;
        pid[1].targetRPM = 0;
    }

    setMotorDirection(pid[1].targetRPM, pid[0].targetRPM);

    long rdeltaTicks = currentRCount - lastRCount;
    long ldeltaTicks = currentlCount - lastlCount;
    lastRCount = currentRCount;
    lastlCount = currentlCount;

    // 2. 轉速
    // CHANGE，一圈脈衝數會翻倍 (11 * GEAR_RATIO * 2)
    float pulsesPerRev = TOTAL_PPR * 2.0; 
    // RPM = (脈衝差 / 每圈脈衝) * (1分鐘 / 0.1秒)
    rightRPM = (rdeltaTicks / pulsesPerRev) * 600.0;
    leftRPM = (ldeltaTicks / pulsesPerRev) * 600.0;
    current_rightRPM = (rdeltaTicks / pulsesPerRev) * 600.0;
    current_leftRPM = (ldeltaTicks / pulsesPerRev) * 600.0;

    //P control
    pid[1].pError = abs(pid[1].targetRPM) - abs(current_rightRPM);
    pid[0].pError = abs(pid[0].targetRPM) - abs(current_leftRPM);

    //I control
    pid[1].iError = abs(pid[1].targetRPM) - abs(current_rightRPM); // 累加誤差
    pid[0].iError = abs(pid[0].targetRPM) - abs(current_leftRPM);  // 累加誤差
    rintegralError += pid[1].iError; // 累加誤差
    lintegralError += pid[0].iError; // 累加誤差
    rintegralError = constrain(rintegralError, -150, 150); 
    lintegralError = constrain(lintegralError, -150, 150); 
    
    //D control
    pid[1].dError = pid[1].pError - rLastError;
    pid[0].dError = pid[0].pError - lLastError;
    rLastError = pid[1].pError;
    lLastError = pid[0].pError;

    //Power output
    if (abs(pid[1].targetRPM) < 5) {
        rightPower = 0;
        pid[1].iError = 0; // 重置積分
    } else {
        rightPower = Base_power + (pid[1].pError * pid[1].kp) + (pid[1].iError * pid[1].ki) + (pid[1].dError * pid[1].kd);
    }
    if (abs(pid[0].targetRPM) < 5) {
        leftPower = 0;
        pid[0].iError = 0; // 重置積分
    } else {
        leftPower = Base_power+ 20 + (pid[0].pError * pid[0].kp) + (pid[0].iError * pid[0].ki) + (pid[0].dError * pid[0].kd);
    }

    rightPower = constrain(rightPower, 0, 255);
    leftPower = constrain(leftPower, 0, 255);


    analogWrite(ENA, (pid[1].targetRPM == 0) ? 0 : (int)rightPower);
    analogWrite(ENB, (pid[0].targetRPM == 0) ? 0 : (int)leftPower);
    
    prevTime = currentTime;
  }
}
