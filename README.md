# Gimbol_PID
## How to make your own Gimbal 
El proyecto consiste en realizar un control PID de dos grados de libertad
para que mantenga en todo momento una posición horizontal

## Hardware
Usamos el ESP8266-01 
Servo1_pitch - pin3
Servo2_roll - pin1

## COntrol PID
Sample Time = 8ms

## Close Loop
Angulo de inclinación : MPU6050
con filtro complementario C1= 0.98 y C2= 0.02

Librerias:
  Arduino:
  #include "Wire.h"
  #include <Servo.h>
  #include "math.h"
  #include <PID_v1.h>
  
Python: (opcional)
  OpenCV
  
  
