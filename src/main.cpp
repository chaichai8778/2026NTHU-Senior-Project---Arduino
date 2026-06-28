#include <math.h>
#include <Arduino.h>
String inputString = "";         // string from pi
bool stringComplete = false;     
float targetAngle = 0.0;       
float target_w = 0.0;        // target "car" angular velocity (rad/s)
float turned_Angle = 0.0;

int Base_power = 68;                
float Base_RPM = 25;               
float Base_linerVelocity = 0.1;     
float leftPower = 0;   
float rightPower = 0;  

// PID parameters 
float pid[3] = {3.9, 0.02, 0};// (Kp, Ki, Kd)
float errorAngle = 0;
float l_pid[3] = {1.8, 0.01, 0};// (lKp, lKi, lKd)
float r_pid[3] = {1.8, 0.01, 0};// (rKp, rKi, rKd)
float error_R = 0;
float error_L = 0;
float error_R_integral = 0;
float error_L_integral = 0;

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
float wheel_perimeter = 0.0675*M_PI;  
float wheel_base = 0.195;           
float wheel_radius = 0.0675/2;                
float dt_sec;
float deltaLcount;
float deltaRcount;
float deltaSL;
float deltaSR;



void doRencoder() {
  if (digitalRead(rencoderPinA) == digitalRead(rencoderPinB)) { 
    rCount++;
  } else {
    rCount--;
  }
}

void dolencoder() {
  if (digitalRead(lencoderPinA) == digitalRead(lencoderPinB)) {
    lCount--;
  } else {
    lCount++;
  }
}

//target > 0 forward，target < 0 backward
void setMotorDirection(float rTarget, float lTarget) {
  // right wheel
  digitalWrite(IN1, (rTarget >= 0) ? LOW : HIGH);
  digitalWrite(IN2, (rTarget >= 0) ? HIGH : LOW);
  
  // left wheeldona
  digitalWrite(IN3, (lTarget >= 0) ? LOW : HIGH);
  digitalWrite(IN4, (lTarget >= 0) ? HIGH : LOW);
}


//turnedAngle(degree) > 0  
//formula: acos(1-0.5*pow((mileage), 2)/pow(wheel_base, 2)) * (180.0 / pi)
float turnedAngle(float deltaSL,float deltaSR) {
  double turned_angle = 0;
  turned_angle = (deltaSL - deltaSR) / wheel_base * 180.0 / M_PI;
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
  attachInterrupt(digitalPinToInterrupt(rencoderPinA), doRencoder, RISING);
  attachInterrupt(digitalPinToInterrupt(lencoderPinA), dolencoder, RISING);

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

    leftRPM  = (deltaLcount / TOTAL_PPR) / dt_sec * 60.0;
    rightRPM = (deltaRcount / TOTAL_PPR) / dt_sec * 60.0;

    deltaSL = ((float)deltaLcount / TOTAL_PPR ) * wheel_perimeter;
    deltaSR = ((float)deltaRcount / TOTAL_PPR ) * wheel_perimeter;

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
    turned_Angle = 0.0;
    } 
    
    //-180 < errorAngle < 180
    turned_Angle += turnedAngle(deltaSL, deltaSR);
    errorAngle = targetAngle - turned_Angle; 

    //P control
    //sign of "errorAngle" = "sign of target_w"
    if (abs(errorAngle) > 2.0) {
      target_w = pid[0]*errorAngle;
      target_w = constrain(target_w, -150, 150);
    }
    else {
      target_w = 0;
    }
    
    targetRPM_L = target_w * wheel_base  / ( wheel_radius * 12 );
    targetRPM_R = -target_w * wheel_base  / ( wheel_radius * 12 );
    
    error_R = abs(targetRPM_R) - abs(rightRPM);
    error_L = abs(targetRPM_L) - abs(leftRPM);
    error_R_integral += error_R * dt_sec;
    error_L_integral += error_L * dt_sec; 
 
    if (target_w != 0) {
      setMotorDirection(targetRPM_R, targetRPM_L);
      leftPower = l_pid[0] * error_L + l_pid[1] * error_L_integral;
      rightPower = r_pid[0] * error_R + r_pid[1] * error_R_integral;
      leftPower  = constrain(Base_power + leftPower, 0, 110);
      rightPower = constrain(Base_power + rightPower, 0, 110);
    } else {
      leftPower  = 0;
      rightPower = 0;
      error_R_integral = 0;
      error_L_integral = 0;
    }

    analogWrite(ENA, (int)rightPower);
    analogWrite(ENB, (int)leftPower);
    deltaLcount = 0;
    deltaRcount = 0;
    prevTime = currentTime;
  }
}
