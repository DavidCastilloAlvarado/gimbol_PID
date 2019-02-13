#include "Wire.h"
#include <Servo.h>
#include "math.h"
#include <PID_v1.h>

#include <Ticker.h>  //Ticker Library
Ticker blinker_pitch,blinker_roll;
Servo servo_pitch,servo_roll;  // create servo object to control a servo
int pos, pos1, posr;
int min_servo = 950;//1446;
int max_servo = 2400;
int min_servo1 = 700;
int max_servo1 = 2000;
int sampleT=8;
int pin_pitch = 3;
int pin_roll = 1;
/*--------------------------  PID  ---------------------------------*/
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Define Variables we'll be connecting to
double Setpoint1, Input1, Output1;
/*---------------------------------------------------------------*/
//Specify the links and initial tuning parameters 
double Kp=0.8, Ki=0.18, Kd=0.0;//===Define the constant Kp,Ki,Kd
PID myPID_pitch(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
/*---------------------------------------------------------------*/
//Specify the links and initial tuning parameters  
double Kp1=0.2, Ki1=0.04, Kd1=0.000;//=====Define the constant Kp,Ki,Kd
PID myPID_roll(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);

/*--------------------------MPU 6050-----------------------------*/
// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito
//MPU6050 sensor_ang;
// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
#define SDA 0
#define SCL 2
const int MPU_addr=0x68; // I2C address of the MPU-6050
int16_t ax, ay, az;
int16_t gx, gy, gz, Tmp;
double ax_d, ay_d, az_d;
double  ang_x, ang_y;
float ang_x_prev, ang_y_prev;
long tiempo_prev,tiempo_prev1,tiempo_prev2;
float dt;
int d_max;
float accel_ang_y,accel_ang_x; 
//time out loop count
const int timeout = 200;
void init_MPU_sensor();
void MPU_sensor();
void PID_axiY_pitch();
void PID_axiX_roll();

/*==============================SETUP=================================*/
void setup() { 
  // PID parametros
  Input = ang_y;
  Setpoint = 0;
  Input1 = ang_x;
  Setpoint1 = 0;
  //turn the PID on
  myPID_pitch.SetMode(AUTOMATIC);
  myPID_pitch.SetSampleTime(sampleT);
  myPID_pitch.SetOutputLimits(-500.0,500.0);
  myPID_roll.SetMode(AUTOMATIC);
  myPID_roll.SetSampleTime(sampleT);
  myPID_roll.SetOutputLimits(-500.0,500.0);
  //--------------------
  //Serial.begin(115200);
  // --- Servos----
  servo_pitch.attach(pin_pitch);  // attaches the servo on GIO2 to the servo object
  servo_roll.attach(pin_roll);
  //------MPU------
  init_MPU_sensor();
  //------Timer----
  blinker_pitch.attach_ms(sampleT, PID_axiY_pitch); 
  delay(sampleT/2);
  blinker_roll.attach_ms(sampleT, PID_axiX_roll); 
}
/*==============================LOOP==================================*/
void loop() 
{
  //MPU_sensor();
  //Serial.print(ang_x); Serial.print("\t");Serial.print(ang_y); Serial.print("\t");Serial.print(pos); Serial.println("\t");
  
}

/*====================================================================*/
void init_MPU_sensor(){
  Wire.begin(SDA, SCL);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}
/*====================================================================*/
void MPU_sensor(){
  /* Leer las aceleraciones y velocidades angulares
  sensor_ang.getAcceleration(&ax, &ay, &az);
  sensor_ang.getRotation(&gx, &gy, &gz);
  // Correxion de Offsets*/
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers
  ax=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  ay=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gx=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  ax=ax-1277;
  ay=ay+428;
  az=az+1723;
  gx=gx+637;
  gy=gy-319;
  gz=gz-224;
  dt = (millis()-tiempo_prev)/1000.0;
  tiempo_prev=millis();
  
  //Calcular los ángulos con acelerometro
  accel_ang_x=atan2(ay,sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
  accel_ang_y=atan2(-ax,sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
  
  //Calcular angulo de rotación con giroscopio y filtro complementario  
  ang_x = 0.98*(ang_x_prev+(gx/127)*dt) + 0.02*accel_ang_x;
  ang_y = 0.98*(ang_y_prev+(gy/160)*dt) + 0.02*accel_ang_y;
   
  ang_x_prev=ang_x;
  ang_y_prev=ang_y;
}
/*============================PITCH=====================================*/
void PID_axiY_pitch(){
  
  dt = (millis()-tiempo_prev1);
  tiempo_prev1=millis();
  //Serial.println("Calculo del Usignal pitch"); Serial.println(dt);
  
  /* in steps of 1 degree
  if (pos<180){
    servo_pitch.write(pos);              // tell servo to go to position in variable 'pos'
    posr++;
    pos++;
  }else{
    posr--;
    servo_pitch.write(posr);              // tell servo to go to position in variable 'pos'
    if (posr == 0){
      pos=0;}
  }
  */
  MPU_sensor();
  Input = ang_y;
  myPID_pitch.Compute();
  pos = servo_pitch.readMicroseconds() - Output ;
  pos = constrain(pos, min_servo, max_servo);
  servo_pitch.writeMicroseconds(pos);              // tell servo to go to position in variable
}
/*=============================ROLL====================================*/
void PID_axiX_roll(){
  dt = (millis()-tiempo_prev2);
  tiempo_prev2=millis();
  //Serial.println("Calculo del Usignal roll"); Serial.println(dt);
  MPU_sensor();
  Input1 = ang_x;
  myPID_roll.Compute();
  pos1 = servo_roll.readMicroseconds() + Output1 ;
  pos1 = constrain(pos1, min_servo1, max_servo1);
  servo_roll.writeMicroseconds(pos1);              // tell servo to go to position in variable
}
