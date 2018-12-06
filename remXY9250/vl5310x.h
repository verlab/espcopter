int otoMeasure;
int hoverlimitUp = 750;
int hoverlimitDown = 550;
int otoMeasure_;
int otoMeasure_0;
int changeMeasure;

float hoverFactor  = 0;
float hoverFactor_ = 450;
 
unsigned long otoPreviousMillis = 0;
const long otoInterval = 35;

int takeBattery =0;


void InitVL53L0X(){

  sensor.init();
  delay(10);
  sensor.setAddress((uint8_t)22);
  sensor.setTimeout(500);

  #if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif
}

void vl5310x(){
  
   unsigned long otoCurrentMillis = millis();
  if(otoCurrentMillis - otoPreviousMillis >= otoInterval){
  otoPreviousMillis = otoCurrentMillis;  
  
  otoMeasure_ = sensor.readRangeSingleMillimeters(); 
  
  if((otoMeasure_ < 2001)  && otoMeasure_ > 10  ){
  //float triOtoMeasure = 2*otoMeasure - (otoMeasure * cos(attitude_Radian[0])) - (otoMeasure * cos(attitude_Radian[1]))      ;
  //otoMeasure = otoMeasure - triOtoMeasure;
   otoMeasure = otoMeasure_;

  otoMeasure = (0.85*(float)otoMeasure) + ((1-0.85)*(float)otoMeasure_0);
  otoMeasure_0 = otoMeasure;
  
  if( otoHover == true && vl5310xControl == 1 ){
  if( takeBattery == 0){
  for(int i = 0;i<25;i++){
  oto.compute(3,targetOto,throttle,0,0,otoMeasure);
  hoverFactor_ = map(6*analogRead(A0),3600,4200,475,375) * weightOfDrone;
  Serial.print("hoverFactor_= ");
  Serial.println(hoverFactor_);
  delay(1);
  }
  takeBattery =1; 
  }
  oto.compute(3,targetOto,throttle,0,0,otoMeasure);
  hoverFactor =  constrain(hoverFactor_ + (oto.output/25),0,motorMax);  
 
  }

  } else if (otoMeasure_ < 2  ){
  otoMeasure= targetOto*2;
  }

 


}
 
 if(otoHover == true  && vl5310xControl == 1){
 if( throttle < round(hoverFactor)){
 throttle = throttle +2.5;
 }
 if( throttle > round(hoverFactor)){
 throttle = throttle -2.5;
 }}else{
    oto.SetInit(0,0,0);
    hoverFactor=0;
    takeBattery =0;
    hoverFactor_ =0;
   // vl5310xControl = 0;
  }

}
