#include <util/atomic.h>

#define MOTOR_1_ENCA 19
#define MOTOR_1_ENCB 44
#define MOTOR_1_PWM 8
#define MOTOR_1_IN1 9
#define MOTOR_1_IN2 10

#define MOTOR_2_ENCA 21
#define MOTOR_2_ENCB 50
#define MOTOR_2_PWM 7
#define MOTOR_2_IN1 6
#define MOTOR_2_IN2 5

#define MOTOR_3_ENCA 18
#define MOTOR_3_ENCB 40
#define MOTOR_3_PWM 13
#define MOTOR_3_IN1 12
#define MOTOR_3_IN2 11

#define MOTOR_4_ENCA 20
#define MOTOR_4_ENCB 46
#define MOTOR_4_PWM 2
#define MOTOR_4_IN1 4
#define MOTOR_4_IN2 3

float linear_x = 0.0;
float linear_y = 0.0;
float angular_z = 0.0;

const float WHEEL_RADIUS = 5;
const float WHEEL_SEPARATION_WIDTH = 180;
const float WHEEL_SEPARATION_LENGTH = 160;

volatile long Motor_1_Pos = 0;
volatile long Motor_2_Pos = 0;
volatile long Motor_3_Pos = 0;
volatile long Motor_4_Pos = 0;

const int rotation_pos = 350 / 2;
long prevT = 0;

long target_m1 = 0;
long target_m2 = 0;
long target_m3 = 0;
long target_m4 = 0;

float eprev_1 = 0;
float eintegral_1 = 0;
float eprev_2 = 0;
float eintegral_2 = 0;
float eprev_3 = 0;
float eintegral_3 = 0;
float eprev_4 = 0;
float eintegral_4 = 0;

// PID constants
float kp = 1;
float kd = 0;
float ki = 0;

void kinematics(float linear_x, float linear_y, float angular_z, float deltaT){
  

  float wheel_front_left  = (1/WHEEL_RADIUS) * (linear_x - linear_y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * angular_z);
  float wheel_front_right = (1/WHEEL_RADIUS) * (linear_x + linear_y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * angular_z);
  float wheel_rear_left   = (1/WHEEL_RADIUS) * (linear_x + linear_y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * angular_z);
  float wheel_rear_right  = (1/WHEEL_RADIUS) * (linear_x - linear_y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * angular_z);
  /*
  Serial.print(wheel_front_left);
  Serial.write("\t");
  Serial.print(wheel_front_right);
  Serial.write("\t");
  Serial.print(wheel_rear_left);
  Serial.write("\t");
  Serial.print(wheel_rear_right);
  Serial.write("\t\n");
  */

  //target_m1 = wheel_front_right * rotation_pos + target_m1;
  //target_m2 = wheel_rear_right * rotation_pos + target_m2;
  //target_m3 = wheel_front_left * rotation_pos + target_m3;
  //target_m4 = wheel_rear_left * rotation_pos + target_m4;

  float m1_ang_vel = ((float)Motor_1_Pos / rotation_pos) / deltaT;
  float m2_ang_vel = ((float)Motor_2_Pos / rotation_pos) / deltaT;
  float m3_ang_vel = ((float)Motor_3_Pos / rotation_pos) / deltaT;
  float m4_ang_vel = ((float)Motor_4_Pos / rotation_pos) / deltaT;

  /*
  Serial.print("Real Vel ");
  Serial.print(m1_ang_vel);
  Serial.print(",");

  Serial.print("Target Vel ");
  Serial.print(wheel_front_right);
  Serial.print(",");

  Serial.print("Pos ");
  Serial.print(Motor_1_Pos);
  Serial.print(",");

  Serial.print("DeltaT ");
  Serial.print(deltaT);
  Serial.print(",");
  */
  
  Motor_1_Pos = 0;
  Motor_2_Pos = 0;
  Motor_3_Pos = 0;
  Motor_4_Pos = 0;
  
  float e1 = m1_ang_vel - wheel_front_right;
  float e2 = m2_ang_vel - wheel_rear_right;
  float e3 = m3_ang_vel - wheel_front_left;
  float e4 = m4_ang_vel - wheel_rear_left;

  /*
  Serial.print("Error");
  Serial.print(e1);
  Serial.print(",");
  */
  
  float dedt1 = (e1-eprev_1)/(deltaT);
  float dedt2 = (e2-eprev_2)/(deltaT);
  float dedt3 = (e3-eprev_3)/(deltaT);
  float dedt4 = (e4-eprev_4)/(deltaT);

  eprev_1 = e1;
  eprev_2 = e2;
  eprev_3 = e3;
  eprev_4 = e4;
  
  eintegral_1 = eintegral_1 + e1*deltaT;
  eintegral_2 = eintegral_2 + e2*deltaT;
  eintegral_3 = eintegral_3 + e3*deltaT;
  eintegral_4 = eintegral_4 + e4*deltaT;

  float u1 = 255 * (kp*e1 + kd*dedt1 + ki*eintegral_1);
  float u2 = 255 * (kp*e2 + kd*dedt2 + ki*eintegral_2);
  float u3 = 255 * (kp*e3 + kd*dedt3 + ki*eintegral_3);
  float u4 = 255 * (kp*e4 + kd*dedt4 + ki*eintegral_4);

  /*
  Serial.print("PWM ");
  Serial.println(u1);
  Serial.print(",");
  
 
  Serial.print("M1 ");
  Serial.print(Motor_1_Pos);
  Serial.print(" M2 ");
  Serial.print(Motor_2_Pos);
  Serial.print(" M3 ");
  Serial.print(Motor_3_Pos);
  Serial.print(" M4 ");
  Serial.println(Motor_4_Pos);
  */
  
  Motor_1_Driver(u1, 1);
  Motor_2_Driver(u2, 1);
  Motor_3_Driver(u3, 1);
  Motor_4_Driver(u4, 1);
}
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
  pinMode(MOTOR_1_IN1, OUTPUT);
  pinMode(MOTOR_1_IN2, OUTPUT);

  pinMode(MOTOR_2_IN1, OUTPUT);
  pinMode(MOTOR_2_IN2, OUTPUT);

  pinMode(MOTOR_3_IN1, OUTPUT);
  pinMode(MOTOR_3_IN2, OUTPUT);

  pinMode(MOTOR_4_IN1, OUTPUT);
  pinMode(MOTOR_4_IN2, OUTPUT);

  pinMode(MOTOR_1_ENCA,INPUT);
  pinMode(MOTOR_1_ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(MOTOR_1_ENCA),Read_Encoder_1,RISING);

  pinMode(MOTOR_2_ENCA,INPUT);
  pinMode(MOTOR_2_ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(MOTOR_2_ENCA),Read_Encoder_2,RISING);

  pinMode(MOTOR_3_ENCA,INPUT);
  pinMode(MOTOR_3_ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(MOTOR_3_ENCA),Read_Encoder_3,RISING);

  pinMode(MOTOR_4_ENCA,INPUT);
  pinMode(MOTOR_4_ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(MOTOR_4_ENCA),Read_Encoder_4,RISING);
}



void loop() 
{
  // collect chars;
  char float_buffer[3][12];
  boolean stringComplete = false;
  int index = 0;
  int idx = 0;


  while (false == stringComplete)
  {
    delay(10);
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;
    kinematics(linear_x, linear_y, angular_z, deltaT);
    
    if (Serial.available() > 0)
    {
      char a= Serial.read();
      if(','== a) 
      {
        float_buffer[index][idx++] = '\0';
        idx = 0;
        index++;
      }
      else if('\n' == a)
      {
          float_buffer[index][idx++] = '\0';
          stringComplete = true;
          linear_x = atof(float_buffer[0]);
          linear_y = atof(float_buffer[1]);
          angular_z = atof(float_buffer[2]);
          //Serial.println(linear_x);
          //Serial.println(linear_y);
          //Serial.println(angular_z);
          
      }
      else
      {
        float_buffer[index][idx++] = a;
      }
    }
  }
}

void Read_Encoder_1(){
  int b = digitalRead(MOTOR_1_ENCB);
  if(b > 0){
    Motor_1_Pos++;
  }
  else{
    Motor_1_Pos--;
  }
}

void Read_Encoder_2(){
  int b = digitalRead(MOTOR_2_ENCB);
  if(b > 0){
    Motor_2_Pos++;
  }
  else{
    Motor_2_Pos--;
  }
}

void Read_Encoder_3(){
  int b = digitalRead(MOTOR_3_ENCB);
  if(b > 0){
    Motor_3_Pos++;
  }
  else{
    Motor_3_Pos--;
  }
}

void Read_Encoder_4(){
  int b = digitalRead(MOTOR_4_ENCB);
  if(b > 0){
    Motor_4_Pos--;
  }
  else{
    Motor_4_Pos++;
  }
}

void Motor_1_Driver(float V, float Vmax) {
  int PWMval = int(abs(V) / Vmax);
  if (PWMval > 255) {
    PWMval = 255;
  }
  if (V > 0) {
    digitalWrite(MOTOR_1_IN1, HIGH);
    digitalWrite(MOTOR_1_IN2, LOW);
  }
  else if (V < 0) {
    digitalWrite(MOTOR_1_IN1, LOW);
    digitalWrite(MOTOR_1_IN2, HIGH);
  }
  else {
    digitalWrite(MOTOR_1_IN1, LOW);
    digitalWrite(MOTOR_1_IN2, LOW);
  }
  analogWrite(MOTOR_1_PWM, PWMval);
}

void Motor_2_Driver(float V, float Vmax) {
  int PWMval = int(abs(V) / Vmax);
  if (PWMval > 255) {
    PWMval = 255;
  }
  if (V > 0) {
    digitalWrite(MOTOR_2_IN1, HIGH);
    digitalWrite(MOTOR_2_IN2, LOW);
  }
  else if (V < 0) {
    digitalWrite(MOTOR_2_IN1, LOW);
    digitalWrite(MOTOR_2_IN2, HIGH);
  }
  else {
    digitalWrite(MOTOR_2_IN1, LOW);
    digitalWrite(MOTOR_2_IN2, LOW);
  }
  analogWrite(MOTOR_2_PWM, PWMval);
}

void Motor_3_Driver(float V, float Vmax) {
  int PWMval = int(abs(V) / Vmax);
  if (PWMval > 255) {
    PWMval = 255;
  }
  if (V > 0) {
    digitalWrite(MOTOR_3_IN1, HIGH);
    digitalWrite(MOTOR_3_IN2, LOW);
  }
  else if (V < 0) {
    digitalWrite(MOTOR_3_IN1, LOW);
    digitalWrite(MOTOR_3_IN2, HIGH);
  }
  else {
    digitalWrite(MOTOR_3_IN1, LOW);
    digitalWrite(MOTOR_3_IN2, LOW);
  }
  analogWrite(MOTOR_3_PWM, PWMval);
}

void Motor_4_Driver(float V, float Vmax) {
  int PWMval = int(abs(V) / Vmax);
  if (PWMval > 255) {
    PWMval = 255;
  }
  if (V > 0) {
    digitalWrite(MOTOR_4_IN1, HIGH);
    digitalWrite(MOTOR_4_IN2, LOW);
  }
  else if (V < 0) {
    digitalWrite(MOTOR_4_IN1, LOW);
    digitalWrite(MOTOR_4_IN2, HIGH);
  }
  else {
    digitalWrite(MOTOR_4_IN1, LOW);
    digitalWrite(MOTOR_4_IN2, LOW);
  }
  analogWrite(MOTOR_4_PWM, PWMval);
}
