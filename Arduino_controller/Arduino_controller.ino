#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// int prevod(int cislo)
// {
//   int nahoru = 520;
//   int dolu = 480;

//   if (cislo > nahoru)
//   {
//     cislo = map(cislo,nahoru,1000,0,1000);
//   }

//   else if (cislo < dolu)
//   {
//     cislo = map(cislo,0,dolu,0,-1000);
//   }

//   return cislo;
// }

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
int plyn;
int naklonX;
int naklonY;
int smerovkaX;

long loopTimer;

// #include <LiquidCrystal_I2C.h>

// LiquidCrystal_I2C lcd(0x27,20,4);

int allData[7];
int newData;
int count = 0;
int dataToTransmit[8];

void setup() 
{
  Serial.begin(115200);

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.stopListening();

  // lcd.init();
  // lcd.backlight();
}

void loop() 
{
  loopTimer = micros();
  if(Serial.available() > 0)
  {      
    newData = Serial.read();
    if(newData == 255)
    {
      while(true)
      {
        newData = Serial.read();       
        if(newData != 255 && newData != 254)
        {
          allData[count] = newData;
          count++;
        }
        if(newData == 254)
        {
          break;
        }
      }
    }
  }
  count = 0;

  plyn = map(analogRead(A0),0,1023,13,180);
  naklonX = map(analogRead(A3),0,1023,0,180);
  naklonY = map(analogRead(A2),0,1023,0,180);
  smerovkaX = map(analogRead(A1),0,1023,0,180);

  // naklonX = prevod(naklonX);
  // naklonY = prevod(naklonY);
  // smerovkaX = prevod(smerovkaX);


  // if(allData[6] == 252) //252 = manual
  // {
  //   dataToTransmit[0] = 255;
  //   dataToTransmit[1] = plyn;
  //   dataToTransmit[2] = 
  //   dataToTransmit[3] = 
  //   dataToTransmit[4] = 
  //   dataToTransmit[5] = 
  //   dataToTransmit[6] = allData[5];
  //   dataToTransmit[7] = 254;
  // }
  if(allData[6] == 253) // 253 = track
  {
    dataToTransmit[0] = 255;
    dataToTransmit[1] = plyn;
    dataToTransmit[2] = allData[0]; //s1
    dataToTransmit[3] = allData[1]; //s2
    dataToTransmit[4] = allData[2]; //s3
    dataToTransmit[5] = allData[3]; //s4
    dataToTransmit[6] = allData[5]; //armed/disarmed
    dataToTransmit[7] = 254;
    // for(int i = 0; i < 8; i++)
    // {
    //   lcd.clear();
    //   lcd.print(String(dataToTransmit[i]));
    //   delay(1000);
    // }
  }
 
  radio.write(&dataToTransmit, sizeof(dataToTransmit));

  while (micros() - loopTimer <= 4000);
  loopTimer = micros();
}