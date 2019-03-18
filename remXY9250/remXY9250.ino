
/*
 * ________________________________________________________________________________________________________
 * metehan son
 * 
*/

#include "global.h"
#include "Parameter.h"
#include <EEPROM.h>
#include <Wire.h>
#include "PID.h"
#include "AHRS.h"

#include "REMOTEXY.h"
#include "WEP_APP.h"
#include "NeoPixel.h"

#include "VL53L0X.h"

VL53L0X sensor;
PID oto;
PID xOpt;
PID yOpt;

PID xMulti;
PID yMulti;

AHRS ahrs;

PID roll;
PID pitch;
PID yaw;

#include "vl5310x.h"
#include "optical.h"
#include "multiRanger.h"

#include "FlightControl.h"
#include "PROCESSING.h"
#include "otoMission.h"


// ************************************************************************************************************************************

void setup() {
  // Set Pin Mode
  Serial.begin(921600);// 921600   
  analogWriteFreq(20000); 
  EEPROM.begin(512);
  scanShields();
  
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(blueLed, OUTPUT);
  
  analogWrite(redLed, PWMRANGE);
  analogWrite(greenLed, PWMRANGE);
  analogWrite(blueLed, 0); 


  pinMode(pinPWM[0], OUTPUT);
  pinMode(pinPWM[1], OUTPUT);
  pinMode(pinPWM[2], OUTPUT);
  pinMode(pinPWM[3], OUTPUT);
   
  analogWrite(pinPWM[0], 0);
  analogWrite(pinPWM[1], 0);
  analogWrite(pinPWM[2], 0);
  analogWrite(pinPWM[3], 0); 

  calAHRS = EEPROM.read(0);

  setupWiFi();
    
   //Start Serial Communication
   ahrs.Initialize(calAHRS);


   roll.SetGain(7.2, 26.0, 0.16,0.6,0.9);// pitch.SetGain(6.2, 22.0, 0.18,0.4,0.9);
   roll.SetLimit(4500, 200, 3500); 
   
   pitch.SetGain(7.2, 26.0, 0.16,0.6,0.9);
   pitch.SetLimit(4500, 200, 3500); 
   
   yaw.SetGain(0.5,0.3,0.6,0.3,0.0);
   yaw.SetLimit(4500, 1000, 6000); 
   
   oto.SetGain(0.0, 0.0, 1.2,5.0,1000.0);  // 1.2,6.8,1000.0); 
   oto.SetLimit(1000, 1000, 6500); 
   
   yOpt.SetGain(0.0, 0.0, 0.1 ,0.3 ,0.0);
   yOpt.SetLimit(4500, 1000, 6000); //3000, 900, 3500
    
   xOpt.SetGain(0.0, 0.0, 0.18 ,0.1,0.0);
   xOpt.SetLimit(4500, 1000, 6000); //3000, 900, 3500

   xMulti.SetGain(0.0, 0.0, 0.9 ,0.3 ,125.0); //  1.5 ,1.00 ,100);
   xMulti.SetLimit(4500, 1000, 6000); //3000, 900, 3500
    
   yMulti.SetGain(0.0, 0.0, 0.9 ,0.3 ,125.0);
   yMulti.SetLimit(4500, 1000, 6000); //3000, 900, 3500
   
   #ifdef vl53l0x
   if(vl5310xControl == 1){
   InitVL53L0X();
   }
   #endif
   
   #ifdef MULTI_RANGER
   multiRangerSetup();
   #endif
   
   #ifdef NeoPixel
   NeoPixelsetup();
   #endif

   #ifdef otoMission
   setup_();
   #endif
   
   delay(1000);
   
}


void loop() {
    calibrationInt();
    
    getRX();   
    modeControl();
    FlightControl(); 

   #ifdef NeoPixel
    neoPixel();
   #endif


   #ifdef otoMission
   loop_();
   #endif

      
 
} 

void calibrationInt(){
  while(calTest == 1 ){    
    unsigned long calMillis = millis();
    Serial.println(calMillis);
    EEPROM.write(0,1);
    EEPROM.commit();
    analogWrite(redLed, 0);
    delay(50);
    analogWrite(redLed, PWMRANGE);
    delay(50);
    if(calMillis > 2000){
    analogWrite(redLed, PWMRANGE);
    analogWrite(greenLed, 0);
    analogWrite(blueLed, 0); // Blue opened
    EEPROM.write(0, 0);
    EEPROM.commit();
    calTest=0;
    break;
    }
    }
  
}

void scanShields(){
  Wire.begin();
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
      Serial.print("0");
      Serial.print(address);
      Serial.println("  !");

      if(address == 22 || address == 41){
      vl5310xControl = 1;
       Serial.println("  Altidude hold shield was found" + vl5310xControl);
        Serial.println(vl5310xControl);
      }
      nDevices++;
      }
     else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  
}
