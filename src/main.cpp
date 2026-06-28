#include <math.h>
#include <Arduino.h>
String inputString = "";         // string from pi
bool stringComplete = false;     
float targetAngle = 0.0;       
float dynamuctargetAngle = 0.0;
float target_w = 0.0;        // target "car" angular velocity (rad/s)


int Base_power = 52;                
float Base_RPM = 25;               
float Base_linerVelocity = 0.1;     
float leftPower = 0;   
float rightPower = 0;  

// PID parameters 
float pid[3] = {0.4, 0, 0};// (lKp, lKi, lKd, lTargetRPM, lintegralError, lLastError)
float sync_Kp = 0.2; 
float sync_Ki = 0.0;
float sync_integral = 0.0;
float sync_lastError = 0.0;

//right wheel parameters
const int ENA =  5;  //speed control
const int IN1 = 13;  //direction control 1
const int IN2 = 12;  //direction control 2
const int rencoderPinA = 2; 
const int rencoderPinB = 8;
volatile long rCount = 0;   
long lastRCount = 0;        
unsigned long prevTime = 0; 
float rightRPM = 0;         
float targetRPM_R = 0;     

//left wheel parameters
const int ENB = 6;   //speed control
const int IN3 = 11;  //direction control 1
const int IN4 = 10;  //direction control 2
const int lencoderPinA = 3;  
const int lencoderPinB = 7;
volatile long lCount = 0;   
long lastlCount = 0;        
float leftRPM = 0;          
float targetRPM_L = 0;

//motor parameters
const int GEAR_RATIO = 30;  
const int PPR = 11;         
const float TOTAL_PPR = (float)PPR *GEAR_RATIO;

//car parameters
float wheel_perimeter = 0.069*M_PI;  
float wheel_base = 0.1975;           
float milage[2] = {0.0, 0.0};        //[left, right]

float dt_sec;
long deltaLcount;
long deltaRcount;


void doRencoder() {
  if (digitalRead(rencoderPinA) == digitalRead(rencoderPinB)) { 
    rCount++;
  } else {
    rCount--;
  }
}

void dolencoder() {
  if (digitalRead(lencoderPinA) == digitalRead(lencoderPinB)) {
    lCount++;
  } else {
    lCount--;
  }
}

//target > 0 forward，target < 0 backward
void setMotorDirection(float rTarget, float lTarget) {
  // 右輪
  digitalWrite(IN1, (rTarget >= 0) ? LOW : HIGH);
  digitalWrite(IN2, (rTarget >= 0) ? HIGH : LOW);
  
  // 左輪
  digitalWrite(IN3, (lTarget >= 0) ? LOW : HIGH);
  digitalWrite(IN4, (lTarget >= 0) ? HIGH : LOW);
}


//turnedAngle(degree) > 0  
//formula: acos(1-0.5*pow((mileage), 2)/pow(wheel_base, 2)) * (180.0 / pi)
float turnedAngle(float milage[2],float wheel_base) {
  double turned_angle = 0;
  if (milage[0] > milage[1]) {
    turned_angle = acos(1-0.5*pow((milage[0]-milage[1]), 2)/pow(wheel_base, 2))*180/M_PI; 
  }
  else {
    turned_angle = acos(1-0.5*pow((milage[1]-milage[0]), 2)/pow(wheel_base, 2))*180/M_PI;
  }
  return turned_angle;
}

// when get input from pi, read the input
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    if (inChar != '\n') {
      inputString += inChar;
    } else {
      stringComplete = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  inputString.reserve(200); 
  
  //right wheel
  pinMode(rencoderPinA, INPUT_PULLUP);
  pinMode(rencoderPinB, INPUT_PULLUP);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  rCount=0;

  //left wheel
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
  unsigned long currentTime = millis();
  serialEvent();

  if (currentTime - prevTime >= 100) {
    noInterrupts(); 
    deltaLcount = lCount - lastlCount;
    deltaRcount = rCount - lastRCount;
    lastRCount = rCount;
    lastlCount = lCount;
    interrupts();

    dt_sec = (currentTime - prevTime) / 1000.0; //time difference(sec)

    leftRPM  = ((float)deltaLcount / TOTAL_PPR) / dt_sec * 60.0;
    rightRPM = ((float)deltaRcount / TOTAL_PPR) / dt_sec * 60.0;

    // when a complete string is received, parse it and control the robot
    if (stringComplete) {
      inputString.trim(); 
      
      // check if the string starts with "AZ:"
      if (inputString.startsWith("AZ:")) {
        String angleStr = inputString.substring(3);
        targetAngle = angleStr.toFloat();

        //-180 < targetAngle < 180
        if (targetAngle > 180) {
          targetAngle = targetAngle - 360; 
        }
      }
    // reset parameters
    inputString = "";
    stringComplete = false;
    milage[0] = 0.0;
    milage[1] = 0.0;
    } 
    
    //milage, +forward, -backward
    milage[0] += ((float)deltaLcount / (TOTAL_PPR * 2.0)) * wheel_perimeter;
    milage[1] += ((float)deltaRcount / (TOTAL_PPR * 2.0)) * wheel_perimeter;
    
    //-180 < errorAngle < 180
    float errorAngle;
    if (targetAngle >= 0) {
      errorAngle = targetAngle - turnedAngle(milage, wheel_base); 
    }
    else {
      errorAngle = targetAngle + turnedAngle(milage, wheel_base); 
    }

    //P control
    //sign of "errorAngle" = "sign of target_w"
    if (abs(errorAngle) > 2.0) {
      target_w = pid[0]*errorAngle;
    }
    else {
      target_w = 0;
    }
    
    float targetRPM_L = target_w * (wheel_base / 2.0) / wheel_perimeter * 60.0;
    float targetRPM_R = -target_w * (wheel_base / 2.0) / wheel_perimeter * 60.0;
    setMotorDirection(targetRPM_R, targetRPM_L);

    // speed synchronization PID parameters
    float sync_error = abs(rightRPM) - abs(leftRPM);
    sync_integral += sync_error * dt_sec;
    sync_integral = min(max(sync_integral, -50), 50);
    
    float dynamic_offset = sync_Kp * sync_error + sync_Ki * sync_integral ;
 
    if (target_w != 0) {
      leftPower = (Base_power + abs(targetRPM_L) * 1.5) + dynamic_offset;
      rightPower = Base_power + abs(targetRPM_R) * 1.5;
      leftPower  = min(max(leftPower, 0), 130);
      rightPower = min(max(rightPower, 0), 120);
    } else {
      leftPower  = 0;
      rightPower = 0;
      sync_integral = 0;
    }

    analogWrite(ENA, (int)rightPower);
    analogWrite(ENB, (int)leftPower);
    
    prevTime = currentTime;
  }
}
