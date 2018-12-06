#ifdef REMOTE_XY_REMOTE
/* 
   -- New project -- 
    
   This source code of graphical user interface  
   has been generated automatically by RemoteXY editor. 
   To compile this code using RemoteXY library 2.3.3 or later version  
   download by link http://remotexy.com/en/library/ 
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/                    
     - for ANDROID 4.1.1 or later version; 
     - for iOS 1.2.1 or later version; 
     
   This source code is free software; you can redistribute it and/or 
   modify it under the terms of the GNU Lesser General Public 
   License as published by the Free Software Foundation; either 
   version 2.1 of the License, or (at your option) any later version.     
*/ 

////////////////////////////////////////////// 
//        RemoteXY include library          // 
////////////////////////////////////////////// 

// RemoteXY select connection mode and include library  
#define REMOTEXY_MODE__ESP8266WIFI_LIB_POINT
#include <ESP8266WiFi.h> 

#include <RemoteXY.h> 

// RemoteXY connection settings  
#define REMOTEXY_WIFI_SSID "Espcopter"
#define REMOTEXY_WIFI_PASSWORD "12345678" 
#define REMOTEXY_SERVER_PORT 6377 

// RemoteXY configurate   
#pragma pack(push, 1) 
uint8_t RemoteXY_CONF[] = 
  { 255,6,0,51,0,59,0,8,24,0,
  5,16,63,14,32,32,31,26,24,5,
  0,6,15,32,32,31,26,24,2,1,
  42,5,18,8,24,26,31,31,65,82,
  77,0,68,73,83,65,82,77,0,3,
  5,47,17,7,32,31,26,67,5,8,
  55,86,5,31,26,51 }; 
   
// this structure defines all the variables of your control interface  
struct { 

    // input variable
  int8_t joystick_1_x; // =-100..100 x-coordinate joystick position 
  int8_t joystick_1_y; // =-100..100 y-coordinate joystick position 
  int8_t joystick_2_x; // =-100..100 x-coordinate joystick position 
  int8_t joystick_2_y; // =-100..100 y-coordinate joystick position 
  uint8_t switch_1; // =1 if switch ON and =0 if OFF 
  uint8_t select_1; // =0 if select position A, =1 if position B, =2 if position C, ... 

    // output variable
  char text_1[51];  // string UTF8 end zero 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY; 
#pragma pack(pop) 

///////////////////////////////////////////// 
//           END RemoteXY include          // 
///////////////////////////////////////////// 

void timer0_ISR (void){
   // Do some work
   // ts1=millis();

    //Serial.print(ts1);
    // Serial.println();
   
    FlightControl(); 
   // Set-up the next interrupt cycle
  timer0_write(ESP.getCycleCount() + 1600000); //80Mhz -> 80*10^6 = 1 second
}

void setupWiFi()  
{    
  RemoteXY_Init();     
  // TODO you setup code 
  /*
  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(timer0_ISR);
  timer0_write(ESP.getCycleCount() + 1600000); //80Mhz -> 80*10^6 = 1 second
  interrupts();*/
} 





void getRX(){
   // timer0_isr_init();
   // timer0_attachInterrupt(timer0_ISR);
   // timer0_write(ESP.getCycleCount() + 1600000); //80Mhz -> 80*10^6 = 1 second
   //interrupts();
    RemoteXY_Handler(); 
   // timer0_detachInterrupt();
    

    armControl = RemoteXY.switch_1; // arm - disarm 

    RX_throttle = (motorMax/100)*RemoteXY.joystick_2_y;

    flyMode = RemoteXY.select_1;
   // char str[] = "ESPcopter is Ready"; 
   // strcpy  (RemoteXY.text_1, str); 

    RX_roll = RemoteXY.joystick_1_x;

    RX_pitch = RemoteXY.joystick_1_y;

    RX_yaw =RemoteXY.joystick_2_x;
}


#endif 
