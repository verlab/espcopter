#include "PID.h"
#include "Parameter.h"



void PID::SetGain(float kp, float ki, float kpo, float kio,float kdo ){
  KP = kp;
  KI = ki;
  KPO = kpo;
  KIO=kio;
  KDO=kdo;
}
void PID::Setp(int32_t output_){

//P = output_;
}

void PID::SetInit(int32_t p, int32_t d, int32_t i){
P=p;
I=i;
D=d;
}
void PID::SetLimit(int16_t error_limit, int16_t i_limit, int16_t output_limit){
  ERROR_LIMIT = error_limit;
  I_LIMIT = i_limit;
  OUTPUT_LIMIT = output_limit;
}
int PID::GetItermRate(){
 return itermRate;
}

void PID::compute(int whichIs, int32_t TARGET_ANGLE,int32_t THROTTLE,int32_t TRIM,int32_t MEASURED_ANGLE, int32_t MEASURED_RATE ){
  // Calculate delta
  timenow = millis();
  float dt = (float)(timenow - timeprev)/978.;
  timeprev = timenow;
  if(dt > -1){
  // Calculate Angle Error
  int32_t ANGLE_ERROR = (TARGET_ANGLE+TRIM) - MEASURED_ANGLE;
 
  // Limit Error Signal
  ANGLE_ERROR = constrain(ANGLE_ERROR, -ERROR_LIMIT, ERROR_LIMIT);
  
  ITERM += ANGLE_ERROR * (float)dt;
  
  // Calculate I-term
   ITERM = constrain(ITERM, -I_LIMIT, I_LIMIT);

  
   if (THROTTLE <100 ) {
   ITERM  = 0;
    }
      
     // Get Desired Rate
  int32_t TARGET_RATE = ANGLE_ERROR * KP + KI * ITERM; // Calculate P-term

  if(whichIs == 2 &&  TARGET_ANGLE != 0){//whichIs == 2 || whichIs == 3 
    TARGET_RATE = TARGET_ANGLE;
  }
  

  if(whichIs == 5 || whichIs == 4 || whichIs == 3 || whichIs == 6 || whichIs == 7 ){//whichIs == 2 || whichIs == 3 
    TARGET_RATE = TARGET_ANGLE;
  }

  // Calculate Rate Error
  int32_t RATE_ERROR = TARGET_RATE - MEASURED_RATE;

  if(whichIs == 3 ){
   RATE_ERROR = constrain(RATE_ERROR, -100, 100);
  }
   if( whichIs == 6 && whichIs == 7 ){
   RATE_ERROR = constrain(RATE_ERROR, -50, 50);
  }

  //if(whichIs == 5 || whichIs == 4 ||whichIs == 3){
  itermRate= constrain(itermRate, -5000, 5000);
 // }else{
 // itermRate= constrain(itermRate, -500, 500);
  //}  
  // Calculate D-Term
  P = RATE_ERROR * KPO;

  delta = ((MEASURED_RATE - _MEASURED_RATE)/dt);
 
  _MEASURED_RATE= MEASURED_RATE;

  delta = 0.6 * (float)delta + (1. - 0.6) * (float)_delta;

  _delta = delta;

/*
    if(whichIs == 3 ){
      if(abs(RATE_ERROR) < 50){
      KDO = 1250;
        
      }else{
       KDO = 500;
      }
   }
   */

  D = (KDO/100) * delta;
/*
   if(whichIs == 4 && TARGET_ANGLE < 200 && TARGET_ANGLE > -200 && MEASURED_ANGLE > -1000 && MEASURED_ANGLE < 1000 ){
   itermRate += RATE_ERROR * (float)dt; 
   }else if( whichIs == 4){
    P=0;
    D=0;
   }
     
   if(whichIs == 5 && TARGET_ANGLE < 200 && TARGET_ANGLE > -200 && MEASURED_ANGLE > -1000 && MEASURED_ANGLE < 1000 ){
   itermRate += RATE_ERROR * (float)dt; 
   }else if(whichIs == 5){
    P=0;
    D=0;
   }
*/

   
   itermRate  = itermRate + RATE_ERROR * (float)dt;
  

   if (THROTTLE <100 ){
   itermRate = 0;
   }   
   
  /*
   if(whichIs == 3 ){
   KIO = 6;
   if(RATE_ERROR!= 0){
   if(RATE_ERROR > -25){
   KIO =  (float)KIO*(float)(abs(RATE_ERROR))/100;
   itermRate  = (itermRate*KIO_)/KIO;
   KIO_ = KIO;
   }else{
   KIO = 1.0;
   }
   }}*/

 
   
   
   I = itermRate * KIO;
   
  output= P  + I - D; //output= P + I - D;
  
  output = 0.95 * (float)output + (1 - 0.95) * (float)_output;
  _output = output;
  
  output = constrain(output, -OUTPUT_LIMIT, OUTPUT_LIMIT);

/*
  if(whichIs == 0){
  Serial.print(ANGLE_ERROR);
  Serial.print(" , ");
  Serial.print(P);
  Serial.print(" , ");
  Serial.print( I);
  Serial.print(" , ");
  Serial.print(output);
  Serial.println();
  }*/
  /*
  if(whichIs == 6){
  Serial.print(MEASURED_RATE);
  Serial.print(" , ");
  Serial.print(P);
  Serial.print(" , ");
  Serial.print(I);
  Serial.print(" , ");
  Serial.print(KIO);
  Serial.print(" , ");
  Serial.print(D);
  Serial.print(" , ");
  Serial.print(output);
  Serial.println();
  }*/
  }else{
  Serial.print((float)dt);
  Serial.println("Eror PİD");
  }
}
