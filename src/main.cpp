#include <math.h>
#include <Arduino.h>

//input
float targetAngle = 0;       
float car_x = 0;
float car_y = 0;
float target_x = 0;
float target_y = 0;
float target_w = 0.0;        // target "car" angular velocity (degree/s)
float target_v = 0.0;
float turned_Angle = 0.0;

float tagetarray[2] = {90 , 0};
int counter = 0;
int counter2 = 0;

int Base_power = 75;     
float linear_RPM = 0.0;                       
float leftPower = 0;   
float rightPower = 0;  

// PID parameters 
float pid[3] = {4.3, 0.04, 0};// (Kp, Ki, Kd)
float l_pid[3] = {1.6, 0.015, 0};// (lKp, lKi, lKd)
float r_pid[3] = {1.6, 0.015, 0};// (rKp, rKi, rKd)
float error_R = 0;
float error_L = 0;
float error_R_integral = 0;
float error_L_integral = 0;
float errorAngle = 0.0;
float errorAngle_integral = 0.0;
float errordistance = 0.0;

//right wheel parameters
const int ENA =  5;  //speed control
const int IN1 = 13;  //direction control 1
const int IN2 = 12;  //direction control 2
const int rencoderPinA = 2; 
const int rencoderPinB = 8;
volatile long rCount = 0;   
long lastRCount = 0;        
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
float milage[2] = {0.0, 0.0};        
float dt_sec;
float deltaLcount;
float deltaRcount;
float deltaSL;
float deltaSR;
float prevTime_motor = 0;

//ultrasonic sensor parameters
const int trigPin = 9;    
const int echoPin = 4;   
float duration = 0;
float obs_distance = 0; //obs_distance(m)
float prevtime_sensor = 0;

//input
struct __attribute__((packed)) NavigationData {
  int16_t car_x;
  int16_t car_y;
  int16_t target_x;
  int16_t target_y;
};
NavigationData navData;

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
  
  // left wheel
  digitalWrite(IN3, (lTarget >= 0) ? HIGH : LOW);
  digitalWrite(IN4, (lTarget >= 0) ? LOW : HIGH);

}

//turnedAngle(degree) > 0  
//formula: turned_angle = (deltaSL - deltaSR) / wheel_base * 180.0 / M_PI
float turnedAngle(float deltaSL,float deltaSR) {
  double turned_angle = 0;
  turned_angle = (deltaSL - deltaSR) / wheel_base * 180.0 / M_PI;
  return turned_angle;
}

void error_caculate(){
  float dx = target_x - car_x;
  float dy = target_y - car_y;
  if (dx==0 && dy==0){
    errordistance = 0;
    errorAngle = 0;
  }
  else{
    errordistance = sqrt(dx*dx + dy*dy) / 100;
    targetAngle = -atan2(dy, dx) * 180 / M_PI;
    errorAngle = targetAngle - turned_Angle; 
    while (errorAngle > 180.0)  errorAngle -= 360.0;
    while (errorAngle < -180.0) errorAngle += 360.0;
  }
 
}

void setup() {
  Serial.begin(115200);
  
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
  pinMode(trigPin, OUTPUT);      
  pinMode(echoPin, INPUT);   
}

void loop() {
  unsigned long currentTime = millis();
  /*f (Serial.available() >= 8) {
    Serial.readBytes((char*)&navData, 8);
  }*/
  if (Serial.available() >= 0) {
      String inputString = Serial.readStringUntil('\n');
      inputString.trim();
      
      int OneComma  = inputString.indexOf(',');
      int TwoComma = inputString.indexOf(',', OneComma + 1);
      int ThrComma  = inputString.indexOf(',', TwoComma + 1);

      if (OneComma == -1 || TwoComma == -1 || ThrComma == -1) {
        return; 
      }

      String carXStr   = inputString.substring(0, OneComma);
      String carYStr   = inputString.substring(OneComma + 1, TwoComma);
      String targetXStr = inputString.substring(TwoComma + 1, ThrComma);
      String targetYStr = inputString.substring(ThrComma + 1);
      
      if (carXStr.length() == 0 || carYStr.length() == 0 || 
          targetXStr.length() == 0 || targetYStr.length() == 0) {
        return;
      }

      car_x   = carXStr.toFloat();
      car_y   = carYStr.toFloat();
      target_x = targetXStr.toFloat();
      target_y = targetYStr.toFloat();
        
  }

  if (currentTime - prevTime_motor >= 15) {
    //dynamic target angle change every 2 seconds
    /*
    counter++;
    if (counter % 135 == 0) {
      counter2++;
      if (counter2 > 1){
        counter2 = 0;
        //turned_Angle = 0.0;
        error_R_integral = 0;
        error_L_integral = 0;
        targetAngle = tagetarray[counter2];
      }
      else{
        //turned_Angle = 0.0;
        error_R_integral = 0;
        error_L_integral = 0;
        targetAngle = tagetarray[counter2];
      }
    }
    */
    
    noInterrupts(); 
    deltaLcount = lCount - lastlCount;
    deltaRcount = rCount - lastRCount;
    lastRCount = rCount;
    lastlCount = lCount;
    interrupts();

    dt_sec = (currentTime - prevTime_motor) / 1000.0; //time difference(sec)

    leftRPM  = (deltaLcount / TOTAL_PPR) / dt_sec * 60.0;
    rightRPM = (deltaRcount / TOTAL_PPR) / dt_sec * 60.0;

    deltaSL = ((float)deltaLcount / TOTAL_PPR ) * wheel_perimeter;
    deltaSR = ((float)deltaRcount / TOTAL_PPR ) * wheel_perimeter;

    error_caculate();

    //-180 < errorAngle < 180

    turned_Angle += turnedAngle(deltaSL, deltaSR);

    target_v = 1.7 * errordistance;
    target_v = constrain(target_v, 0.0, 0.24);

    float base_linear_RPM = (target_v / wheel_perimeter) * 60.0;

    //P control
    //sign of "errorAngle" = "sign of target_w"
    if (abs(errordistance) > 0.02){
      if (abs(errorAngle) > 2.0) {
        errorAngle_integral += errorAngle * dt_sec;
        errorAngle_integral = constrain(errorAngle_integral, -50.0, 50.0);
        target_w = pid[0]*errorAngle + + pid[1] * errorAngle_integral;;
        target_w = constrain(target_w, -90, 90);
        float linear_factor = 1.0 - pow((min(abs(errorAngle), 180.0) / 180.0),2);
        linear_RPM = base_linear_RPM * linear_factor;
      }
      else {
        target_w = 0;
        errorAngle_integral = 0;
        linear_RPM = base_linear_RPM;
      }
    }
    else
    {
      target_v = 0;
      target_w = 0;
      errorAngle_integral = 0;
    }
    
    
    float rotate_RPM = target_w * wheel_base / (wheel_radius * 12);

    targetRPM_L = rotate_RPM + linear_RPM;
    targetRPM_R = -rotate_RPM + linear_RPM;

    error_R = abs(targetRPM_R) - abs(rightRPM);
    error_L = abs(targetRPM_L) - abs(leftRPM);
    error_R_integral += error_R * dt_sec;
    error_L_integral += error_L * dt_sec;
    error_R_integral = constrain(error_R_integral, -50.0, 50.0);
    error_L_integral = constrain(error_L_integral, -50.0, 50.0);

    if (target_w != 0) {
      setMotorDirection(targetRPM_R, targetRPM_L);
      leftPower = l_pid[0] * error_L + l_pid[1] * error_L_integral;
      rightPower = r_pid[0] * error_R + r_pid[1] * error_R_integral;
      leftPower  = constrain(Base_power + leftPower, 0, 120);
      rightPower = constrain(Base_power + rightPower, 0, 110);
    } else {
      leftPower  = 0;
      rightPower = 0;
      error_R_integral = 0;
      error_L_integral = 0;
    }

    static int sensor_trigger_counter = 0;
    sensor_trigger_counter++;
    
    if (sensor_trigger_counter >= 4) {
      sensor_trigger_counter = 0; 
      
      digitalWrite(trigPin, LOW);
      delayMicroseconds(5);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      pinMode(echoPin, INPUT);
      
      duration = pulseIn(echoPin, HIGH, 11600); 
      
      if (duration > 0) {
        obs_distance = (duration / 2.0) / 29.1; 
      } else {
        obs_distance = 200.0;
      }
    }
/*
    //Ultrasonic sensor
    //obs_distance(m) caculate
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    pinMode(echoPin, INPUT);
    duration = pulseIn(echoPin, HIGH);
    pulseIn(echoPin, HIGH, 30000);
    obs_distance = (duration/2) / 29.1; 
*/
    //Serial output
    Serial.print(obs_distance);    
    Serial.print(",");
    Serial.print(deltaSL, 6);
    Serial.print(",");
    Serial.println(deltaSR, 6);

    analogWrite(ENA, (int)rightPower);
    analogWrite(ENB, (int)leftPower);
    prevTime_motor = currentTime;
    prevtime_sensor = currentTime;
  }
}