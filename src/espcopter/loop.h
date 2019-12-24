
unsigned long previousMillisabc = 0;      
void mainLoop(){
  unsigned long getRXTime = micros();
  //setVl5310xControl(true);
  //setFlyMode_1(true);
  //getRX();   <--- isso vai vir do ros
  modeControl();
  FlightControl();  
  unsigned long getRXTime2 = micros();

    /*
 //  if( getRXTime2 - getRXTime > 500){
   Serial.print("  getRX: ");
   Serial.print( getRXTime2 - getRXTime );
   Serial.println();
   */
}
