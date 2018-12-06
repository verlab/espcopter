
#include "Parameter.h"

unsigned long ts=micros();

unsigned long tsbat=micros();
boolean ledState =   HIGH;

float factor;
float factor_;
void FlightControl(){

if((micros()-tsbat)>100000){  //Update only on 
 tsbat = micros();
// Serial.println(analogRead(A0)*6);
 if(analogRead(A0)*6  < 2900){
  batteryCount= batteryCount+1;

 }else{
 batteryCount= 0;
}
if(batteryCount > 25){
  batteryControl = 0;
}
}

 
if((micros()-ts)>1000){  //Update only once per 2ms (500hz update rate)
  ts = micros();
  ahrs.compute(attitude, rate);  
  
  attitude_Radian[0] = attitude[0]/(100*RAD2DEG);
  attitude_Radian[1] = attitude[1]/(100*RAD2DEG);

  attitude_rate[0] = rate[0]/(100*RAD2DEG);
  attitude_rate[1] = rate[1]/(100*RAD2DEG);
  attitude_rate[2] = rate[2]/(100*RAD2DEG);
  
  ahrs.headingMag(attitude_rate, attitude_Radian, degree , throttle); 
 
  if( yawControl==0){
  degree[0] = 0;
  }

 
  //  Serial.print(((float) ahrs.readTemp()) / 333.87f + 21.0f);
  //  Serial.println(0);

   #ifdef vl53l0x
   if(vl5310xControl == 1){
   vl5310x();
   }
   #endif

   #ifdef opticalFlow
    opticalSensor();
   #endif  
   
   #ifdef MULTI_RANGER
   multiRangerLoop();
   #endif 
      /*
    Serial.print(otoMeasure );
    Serial.print(" , ");
    Serial.print(DistanceX );
    Serial.print(" , ");
    Serial.print(DistanceY  );
    Serial.print(" , ");
    Serial.print(xMulti.output);
    Serial.print(" , ");
    Serial.print(yMulti.output );
    Serial.print(" , ");
    Serial.print( SetPoint[1] );
    Serial.println();
  */
   if(throttle > 2){
   roll.compute( 0 , SetPoint[0] - xMulti.output  ,throttle, -xOpt.output +Trim_Roll  ,-attitude[1], rate[1]); // Trim_Roll xOpt.output
   pitch.compute( 1 , SetPoint[1] - yMulti.output,throttle,  Trim_Pitch  , -attitude[0], -rate[0]);
   yaw.compute( 2 , SetPoint[2] , throttle,Trim_Yaw, degree[0]*100, -rate[2]);//-rate[2]degree[0]*100 
   }else{
   ahrs.setZero();  
   }
   
  }

  if(throttle >= motorMax) {throttle = motorMax;} else if(throttle <= 0) {throttle = 0;}
  
  factor =1+pow(2,throttle/250);// pow(2,throttle/250) ; //+pow(2,throttle/250)
  factor_ = 100;
  motorFL = throttle + (roll.output/factor_)*factor - (pitch.output/factor_)*factor + (yaw.output/factor_)*factor;
  motorFR = throttle - (roll.output/factor_)*factor - (pitch.output/factor_)*factor - (yaw.output/factor_)*factor;
  motorRL = throttle + (roll.output/factor_)*factor + (pitch.output/factor_)*factor - (yaw.output/factor_)*factor;
  motorRR = throttle - (roll.output/factor_)*factor + (pitch.output/factor_)*factor + (yaw.output/factor_)*factor;

  if(motorFL >= PWMRANGE) {motorFL = PWMRANGE;} else if(motorFL <= 0) {motorFL = 0;}
  if(motorFR >= PWMRANGE) {motorFR = PWMRANGE;} else if(motorFR <= 0) {motorFR = 0;}
  if(motorRL >= PWMRANGE) {motorRL = PWMRANGE;} else if(motorRL <= 0) {motorRL = 0;}
  if(motorRR >= PWMRANGE) {motorRR = PWMRANGE;} else if(motorRR <= 0) {motorRR = 0;}

  // Input Control Value to PWM Pins
  if(throttle > 2){
  analogWrite(pinPWM[0], round(motorFL));
  analogWrite(pinPWM[1], round(motorFR));
  analogWrite(pinPWM[2], round(motorRL));
  analogWrite(pinPWM[3], round(motorRR));
  }else{
  analogWrite(pinPWM[0], 0);
  analogWrite(pinPWM[1], 0);
  analogWrite(pinPWM[2], 0);
  analogWrite(pinPWM[3], 0);
  } 
}

void modeControl(){

if(armControl == 1 && batteryControl == 1 && stopFlightControl == 1){
    throttleControl = 1;
   

    if(flyMode_1 == 1){
    yawControl=1;
    throttleControl = 1;
    }else{
    yawControl=0; 
    }
 
    if(flyMode_2 == 1){
    throttleControl = 0;
    otoHover=1;
    }else if(flyMode_3 == 1){
    throttleControl = 0;
    otoHover=1;
    }else{
    otoHover=0;
    }
    
    }else if(batteryControl == 0){
    throttleControl = 0;
    otoHover=0;
    yawControl=0;
    throttle = throttle - 0.5;
    }else if(stopFlightControl == 0){
    throttleControl = 0;
    otoHover=0;
    yawControl=0;
    throttle = 0;
    }else{
    throttleControl = 0;
    otoHover=0;
    yawControl=0;
    throttle=0;
    }

    
    if(throttleControl == 1){
   // throttle= RX_throttle ;
    if( throttle < RX_throttle){
    throttle = throttle +2.5;
    }
    if( throttle > RX_throttle){
    throttle = throttle -2.5;
    }
    }else{
   // targetOto = constrain(RX_throttle,250,500);
    }


   SetPoint[0] = map(RX_roll, -100, 100, -ROLL_LIMIT, ROLL_LIMIT);
   SetPoint[1] = map(RX_pitch, -100, 100, -PITCH_LIMIT, PITCH_LIMIT);

   if(yawControl == 0 ){
   SetPoint[2] = map(RX_yaw, -100, 100, -YAW_LIMIT, YAW_LIMIT);  
   }else{
   SetPoint[2] = 0;
   }

  throttle = constrain(throttle,0, 700);
    
}
