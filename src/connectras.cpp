#include <math.h>
#include <Arduino.h>

String inputString = "";         // 用來儲存從樹莓派收到的字串
bool stringComplete = false;     // 判斷是否收到換行符號 \n（代表一句話結束）
float targetAzimuth = 0.0;       // 解析出來的目標角度數字

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

void setMotorDirection(float rTarget, float lTarget) {
  // 右輪
  digitalWrite(IN1, (rTarget >= 0) ? LOW : HIGH);
  digitalWrite(IN2, (rTarget >= 0) ? HIGH : LOW);
  
  // 左輪
  digitalWrite(IN3, (lTarget >= 0) ? LOW : HIGH);
  digitalWrite(IN4, (lTarget >= 0) ? HIGH : LOW);
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
  else if (targetAngle < -2.0 && targetAngle >= -179.0) {
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

// --- 硬體中斷/通訊緩衝接收函數 ---
// 當 Arduino 的 RX 腳位收到地心引力傳來的訊號時，會自動觸發這個函數
void serialEvent() {
  while (Serial.available()) {
    // 讀取一個新字元
    char inChar = (char)Serial.read();
    
    // 如果不是換行符號，就一直把它貼在屁股後面接下去
    if (inChar != '\n') {
      inputString += inChar;
    } else {
      // 一旦摸到 \n，代表樹莓派這句話說完了
      stringComplete = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  // 預留 200 個位元組給字串，防止樹莓派狂丟資料時記憶體碎片化
  inputString.reserve(200); 
  
  //右輪
  pinMode(rencoderPinA, INPUT_PULLUP);
  pinMode(rencoderPinB, INPUT_PULLUP);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  rCount=0;

  //左輪
  pinMode(lencoderPinA, INPUT_PULLUP);
  pinMode(lencoderPinB, INPUT_PULLUP);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  lCount=0;
  attachInterrupt(digitalPinToInterrupt(rencoderPinA), doRencoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(lencoderPinA), dolencoder, CHANGE);

}

void loop() {
  // 2. 呼叫通訊檢查函數
  serialEvent();

  // 3. 當收到完整的一行資料時（例如收到 "AZ:123.4\n"）
  if (stringComplete) {
    inputString.trim(); // 移除前後不小心的空格或換行符號
    
    // 檢查字串開頭是不是我們約定的 "AZ:"
    if (inputString.startsWith("AZ:")) {
      // 拔掉 "AZ:"，轉換成浮點數 (float)
      String angleStr = inputString.substring(3);
      targetAzimuth = angleStr.toFloat();
      
      // 回傳給樹莓派，讓樹莓派終端機顯示 "🤖 Arduino 回應: ACK: 123.4" 
      // 這能幫你確認兩邊有沒有斷線
      Serial.print("ACK: ");
      Serial.println(targetAzimuth);
      
      // =======================================================
      // 🎯 在這裡接上你原本「知道角度數字就能自己開車轉向」的程式！
      // 你的函數可能長得像： driveRobotToAngle(targetAzimuth);
      // =======================================================
    }
    
    // 處理完後，清空快取，等待下一條求救指令
    inputString = "";
    stringComplete = false;
  }
  
  // 這裡放你原本 loop 裡面的其他例行公事...
}

