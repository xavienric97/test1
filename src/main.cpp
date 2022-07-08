#include <Arduino.h>
#include <Wire.h> //This is for i2C
#include <AS5600.h>
#include <HardwareSerial.h>
#include <PID_v1.h>
#include <cstdlib>

using namespace std;
// // I2C pins:
// // STM32: SDA: PB7 --- SCL: PB6

// // Serial Print Pins:
// // PA3: RX2 --- PA2: TX2

// Stepper Motor: 12V
// In2: CCW Positive PB8
// In1: CW Negative PB9

//////////////////////////

// Error = 0 dentro del rango aceptable
// 90 to 270
// Actual movement: {{530, 530}, {570, 560}, {0, 0}, {530, 530}, {0,0}}

HardwareSerial Serial2(PA3, PA2); // Para el serial print con un Arduino UNO

int32_t motors[5][4] = {{530, 530, 1023, -1023}, {530, 530, 1023, -1023}, {530, 530, 1023, -1023}, {560, 560, 1023, -1023}, {0,0, 1023, -1023}};
int32_t motor_num = 3;
int32_t dir, dead_zone_up = motors[motor_num][0], dead_zone_down = motors[motor_num][1];
float error, tolerancia, tol_up, tol_down;

double convertRawAngleToDegrees(word);
int32_t direction(float diferencia);

void position_correction(float lectura, int32_t val, int32_t direccion);

double Setpoint, Input, Output, Final_Time, Previous_Time, Previous_Angle;
bool done = false;
static double KP=25, KI=0, KD=0;
PID myPID(&Input, &Output, &Setpoint, KP, KI, KD, DIRECT);
AMS_5600 ams5600;

int ang, lang = 0;

void setup()
{
  Serial2.begin(74880);
  Wire.begin();
  pinMode(PB8, OUTPUT); //Pins para el ouput PWM del motor
  pinMode(PB9, OUTPUT); //^^^
  analogWriteResolution(10); //Resolucion de 1023
  Setpoint = 180;
  Input = convertRawAngleToDegrees(ams5600.getRawAngle());
  Previous_Angle = Input;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(20);
  myPID.SetOutputLimits(-1023 + dead_zone_down, 1023-dead_zone_up);
  analogWriteFrequency(20000);
}

void loop()
{
    // Deteccion del magneto por el enoder
    if(ams5600.detectMagnet() == 0){
      while(1){
        if(ams5600.detectMagnet() == 1){ break; }
        else{ Serial2.println("Can not detect magnet"); }
      }
    }

    // Visualizar la lectura del angulo
    Serial2.println("------------------------------");
    Input = convertRawAngleToDegrees(ams5600.getRawAngle());

    tolerancia = 1;
    tol_up = Setpoint+tolerancia;
    tol_down = Setpoint-tolerancia;
    error = Input - Setpoint;

    myPID.Compute();
    Serial2.println(Input);
    Serial2.println(Setpoint);
    Serial2.println(Output);
    position_correction(Input, Output, direction(error));
}

int32_t direction(float diferencia)
{
  if (diferencia > 0){ dir = 0; }
  else if (diferencia < 0){ dir = 1; }
  return dir;
}

void position_correction(float lectura, int32_t val, int32_t direccion)
{
  if(lectura > tol_down && lectura < tol_up)
  {
    analogWrite(PB8, 0);
    analogWrite(PB9, 0);
    if(!done){
      Final_Time = millis() - Previous_Time;
      done = true;
    }
    else if(done && millis() > Final_Time + Previous_Time + 5000){
      Serial2.print("Final Time:");
      Serial2.println(Final_Time/1000);
      Serial2.print("Current Time:");
      Serial2.println(millis()/1000);
      Serial2.print(abs(Setpoint - Previous_Angle));
      Serial2.println(" degree change");
      while(Serial2.available() == 0){}
      Setpoint = Serial2.readString().toInt();
      Previous_Angle = Setpoint;
      Previous_Time = millis();
    }
  }
  else if (val  > 0)
  {
    val += dead_zone_up;
    if(val > 1023)
    {
      val = 1023;
    }

    // Serial2.println(val);
    analogWrite(PB9, 0);
    analogWrite(PB8, val);
    // Serial2.println("CCW or Positive");

    if(done){ done = false;}
  }
  else
  {
    val -= dead_zone_down;
    if(val < -1023)
    {
      val = -1023;
    }
    // Serial2.println(val);
    analogWrite(PB8, 0);
    analogWrite(PB9, -val);
    // Serial2.println("CW or Negative");

    if(done){ done = false;}
  }
}


double convertRawAngleToDegrees(word newAngle)
{
  if (newAngle == 0xFFFF) return 0xFFFF;
  /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
  float retVal = newAngle * 0.087890625;
  return retVal;
}