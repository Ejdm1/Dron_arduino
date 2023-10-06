#define pinTrigger    41
#define pinEcho       43
#define maxVzdalenost 400

int vzdalenost = 10000;
unsigned long posl = 0;
unsigned long predProg = 0;
unsigned long jedS = 0;
unsigned long behemProg = 0;
unsigned long rozdil = 0;
int inter = 1000;
int naklon = 0;
int bzucPin1 = 47;
int senzorTlac1 = 46;
int pomSetup = true;
int pomProg = true;
unsigned long timer = 0;
float offset = 0;
long loopTimer, loopTimer2;
int temperature;
double accelPitch;
double accelRoll;
long acc_x, acc_y, acc_z;
double accel_x, accel_y, accel_z;
double gyroRoll, gyroPitch, gyroYaw;
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
double rotation_x, rotation_y, rotation_z;
double freq, dt;
double tau = 0.98;
double roll = 0;
double pitch = 0;
// 250 deg/s --> 131.0, 500 deg/s --> 65.5, 1000 deg/s --> 32.8, 2000 deg/s --> 16.4
long scaleFactorGyro = 65.5;
// 2g --> 16384 , 4g --> 8192 , 8g --> 4096, 16g --> 2048
long scaleFactorAccel = 8192;

int plyn = 0;

const byte rxPin = 31;
const byte txPin = 30;

#include <NewPing.h>
#include "Wire.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <SoftwareSerial.h>


SoftwareSerial radio2(rxPin, txPin);

NewPing sonar(pinTrigger, pinEcho, maxVzdalenost);

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

int data[8];

Servo motor;

void setup() 
{
  // put your setup code here, to run once:
  pinMode(bzucPin1, OUTPUT);
  pinMode(senzorTlac1, INPUT);
  digitalWrite(bzucPin1, HIGH);
  pinMode(37,OUTPUT);
  motor.attach(37,1000,2000);
  // Serial1.begin(115200);
  // Serial2.begin(115200);
  Serial.begin(115200);
  radio2.begin(115200);
  // Serial1.begin(152000);
  // Serial1.print("AT+ADDRES=0");
  // Serial1.flush();
  while(pomSetup == true)
  {
    if(digitalRead(senzorTlac1) == LOW)
    {
      digitalWrite(bzucPin1, LOW);
      delay(1000);
      digitalWrite(bzucPin1, HIGH);
      // Start

      radio.begin();
      radio.openReadingPipe(0, address);
      radio.setPALevel(RF24_PA_MIN);
      radio.setDataRate(RF24_250KBPS);
      radio.startListening();
      Wire.begin();
      
      // Setup the registers of the MPU-6050 and start up
      setup_mpu_6050_registers();
      Serial.println("Calibrating gyro, place on level surface and do not move.");
      for (int cal_int = 0; cal_int < 3000; cal_int ++)
      {
        if(cal_int % 200 == 0)Serial.print(".");
          read_mpu_6050_data();
          gyro_x_cal += gyro_x;
          gyro_y_cal += gyro_y;
          gyro_z_cal += gyro_z;
          delay(3);
        }
      gyro_x_cal /= 3000;
      gyro_y_cal /= 3000;
      gyro_z_cal /= 3000;

      delay(500);
      digitalWrite(bzucPin1, LOW);
      delay(1500);
      digitalWrite(bzucPin1, HIGH);
      pomSetup = false;
    }
  }
}

void loop() 
{
  Serial.println("main started");
  // put your main code here, to run repeatedly:
  while(true)
  {
    predProg = millis();

    while(pomProg == true)
    {
      loopTimer = micros();
      loopTimer2 = micros();
      freq = 1/((micros() - loopTimer2) * 1e-6);
      loopTimer2 = micros();
      dt = 1/freq;
      read_mpu_6050_data();
      gyro_x -= gyro_x_cal;
      gyro_y -= gyro_y_cal;
      gyro_z -= gyro_z_cal;
      rotation_x = (double)gyro_x / (double)scaleFactorGyro;
      rotation_y = (double)gyro_y / (double)scaleFactorGyro;
      rotation_z = (double)gyro_z / (double)scaleFactorGyro;
      accel_x = (double)acc_x / (double)scaleFactorAccel;
      accel_y = (double)acc_y / (double)scaleFactorAccel;
      accel_z = (double)acc_z / (double)scaleFactorAccel;

      accelPitch = atan2(accel_y, accel_z) * RAD_TO_DEG;
      accelRoll = atan2(accel_x, accel_z) * RAD_TO_DEG;
      pitch = (tau)*(pitch + rotation_x*dt) + (1-tau)*(accelPitch);
      roll = (tau)*(roll - rotation_y*dt) + (1-tau)*(accelRoll);
      gyroPitch += rotation_x*dt;
      gyroRoll -= rotation_y*dt;
      gyroYaw += rotation_z*dt;

      if (radio.available()>0)
      {
        //char text[32] = "";
        radio.read(&data, sizeof(data));
        // radio.flush_rx();
        //radio.read(&text, sizeof(text));

        // String veta = String(text);
        // int arm = (veta.substring(veta.indexOf("a")+1,veta.indexOf("a") + 2)).toInt();
        // int servoX = (veta.substring(veta.indexOf("o")+1,veta.indexOf(","))).toInt();
        // int servoY = (veta.substring(veta.indexOf(",")+1,veta.indexOf("x"))).toInt();
        // int naklonX = (veta.substring(veta.indexOf("x") + 1,veta.indexOf("y"))).toInt();
        // int naklonY = (veta.substring(veta.indexOf("y")+1,veta.indexOf("s"))).toInt();
        // int smerovka = (veta.substring(veta.indexOf("s")+1,veta.indexOf("p"))).toInt();
        // int plyn = (veta.substring(veta.indexOf("p") + 1,veta.length())).toInt();
        if(data[0] == 255 && data[7] == 254)
        {
          plyn = data[1];
          int servo1 = data[2];
          int servo2 = data[3];
          int servo3 = data[4];
          int servo4 = data[5];
          bool armed = data[6];
        }

        String toPrint = "";
        for(int i = 0;i < 8;i++)
        {
          toPrint = toPrint + String(data[i]) + " ";
        }
        Serial.println(toPrint);

        //Serial.println(plyn);
        motor.write(0);
      }
      else
      {
        Serial.println("nothing yet");
      }

        // if (millis() > posl + inter)
        // {
        //   radio2.print("AT");
        //   Serial.println("pppppppppoossssssllll");
        //   posl = millis();
        // }
        // if (radio2.available()){
        //   Serial.println(radio2.readString());
        // }

      rozdil = behemProg - predProg;
      if ((rozdil %= 50) < 8 && (millis() - jedS) > 40)
      {
        vzdalenost = sonar.ping_cm();
        // Serial.print(pitch);
        // Serial.print(",");
        // Serial.print(roll);
        // Serial.print("  ");
        //------------------------------Serial.println(vzdalenost);
        // Serial.println();
        // if(Serial1.available()){
        //   prichozi = Serial1.readString();
        //   Serial.print(prichozi);
        // }
        // Serial1.flush();

        jedS = millis();
      }

      // Wait until the loopTimer reaches 4000us (250Hz) before next loop
      while (micros() - loopTimer <= 4000);
      loopTimer = micros();
      behemProg = millis();
    }
  }
}

void read_mpu_6050_data() 
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);

  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() <<8 | Wire.read();
  gyro_x = Wire.read()<<8 | Wire.read();
  gyro_y = Wire.read()<<8 | Wire.read();
  gyro_z = Wire.read()<<8 | Wire.read();
}

void setup_mpu_6050_registers() 
{
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Configure the accelerometer
  // Wire.write(0x__);
  // Wire.write; 2g --> 0x00, 4g --> 0x08, 8g --> 0x10, 16g --> 0x18
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission();

  // Configure the gyro
  // Wire.write(0x__);
  // 250 deg/s --> 0x00, 500 deg/s --> 0x08, 1000 deg/s --> 0x10, 2000 deg/s --> 0x18
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}