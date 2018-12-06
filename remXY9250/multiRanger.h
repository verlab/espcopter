

#ifdef MULTI_RANGER

VL53L0X sensor1, sensor2;
float DistanceX, DistanceY;
float DistanceX_, DistanceY_;
float DistanceX_0, DistanceY_0;
int targetMultiX=500;
int targetMultiY=500;




#define TCAADDR 0x70

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

unsigned long multiPreviousMillis = 0;
const long multiInterval = 35;


void multiRangerSetup(){

  tcaselect(1);
  sensor1.init();
  sensor1.setTimeout(500);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor1.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  //sensor1.setMeasurementTimingBudget(200000);
#endif

  tcaselect(2);
  sensor2.init();
  sensor2.setTimeout(500);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor2.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

 // sensor2.setMeasurementTimingBudget(200000);
#endif


  Wire.beginTransmission(TCAADDR);
  Wire.write(0);
  Wire.endTransmission();
  
}

void multiRangerLoop(){

  unsigned long multiCurrentMillis = millis();
  if(multiCurrentMillis - multiPreviousMillis >= multiInterval){
  multiPreviousMillis = multiCurrentMillis;  

  unsigned long multiCurrentMillis_1 = micros();
  tcaselect(1); 
  int val1 = sensor1.readRangeSingleMillimeters();

  tcaselect(2); 
  int val2 = sensor2.readRangeSingleMillimeters();

  Wire.beginTransmission(TCAADDR);
  Wire.write(0);
  Wire.endTransmission();

 unsigned long multiCurrentMillis_2 = micros();
 
  if((val1 < 2001)  &&  (val1 > 10) ){
  DistanceY = val1;
  }else{
  DistanceY= targetMultiY*1.5;
  }

  if((val2 < 2001)  &&  (val2 > 10) ){
  DistanceX = val2;
  }else{
  DistanceX= targetMultiX*1.5;
  }

  Serial.print("DistanceY 1 = ");
  Serial.print(DistanceY);
  Serial.print(" DistanceX 1= ");
  Serial.print(DistanceX);


 
 DistanceX = (0.85*(float)DistanceX) + ((1-0.85)*(float)DistanceX_);
/*
if(abs(DistanceX - DistanceX_) > 50){
  DistanceX = targetMultiX;
}*/
 DistanceX_ = DistanceX;

 DistanceY = (0.85*(float)DistanceY) + ((1-0.85)*(float)DistanceY_);
/*
if(abs(DistanceY - DistanceY_) > 50){
  DistanceY = targetMultiY;
}
*/
 
 DistanceY_ = DistanceY;


  Serial.print(" DistanceY= ");
  Serial.print(DistanceY);
  Serial.print(" DistanceX= ");
  Serial.println(DistanceX);

 
   
if(otoMeasure > 60){
  xMulti.compute(6,targetMultiX,throttle,0,0,DistanceX);
  yMulti.compute(7,targetMultiY,throttle,0,0,DistanceY);
}else{
  xMulti.compute(6,targetMultiX,throttle,0,0,targetMultiX);
  yMulti.compute(7,targetMultiY,throttle,0,0,targetMultiX);
}
 

  }

  
}
#endif  
