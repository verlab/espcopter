#ifdef opticalFlow

int opticalUpdate = 50;
unsigned long opticalTs=millis();


 int limitOpt = 20;
 int16_t deltaY = 0, deltaX = 0 ,deltaX_= 0, deltaY_= 0, deltaCsr =0;
 int _flow_sum_x = 0;
 int _flow_sum_y = 0;

 int delta_x = 0;
 int delta_y = 0;
 
void opticalSensor() {
  

 if((millis()-opticalTs)>opticalUpdate   ){  //Update only once per 10sec   && flyMode == 2 
 opticalTs = millis();

 Wire.requestFrom(8, 6);    // request 6 bytes from slave device #8
 int16_t response[6];
 int index = 0;
 // Wait for response
 while (Wire.available()) {
 int16_t b = Wire.read();
 response[index] = b;
 index++;
 }
  
  deltaY =word(response[0], response[1]);
  deltaX =word(response[2], response[3]);
  deltaCsr = word(response[4], response[5]);


  if( (deltaY + deltaX) == deltaCsr){

   analogWrite(2, 0);
  }else{
    deltaY =0;
    deltaX =0;
    analogWrite(2, PWMRANGE);
  }
  if (isnan(deltaY)) {
    deltaY=0;
}

  if (isnan(deltaX)) {
    deltaX=0;
}
/*
  deltaY = (0.6*(float)deltaY) + (1-0.6)*(float)deltaY_;
  deltaY_ = deltaY;
 
  deltaX = (0.6*(float)deltaX) + (1-0.6)*(float)deltaX_;
  deltaX_ = deltaX;
  */
  
  // deltaX = constrain(deltaX, -limitOpt, limitOpt);
 //  deltaY = constrain(deltaY, -limitOpt, limitOpt);


  // _flow_sum_x += deltaX;
  // _flow_sum_y += deltaY;

  //  delta_x = (float)_flow_sum_x / 500.0f;    // proportional factor + convert from pixels to radians
  //  delta_y = (float)_flow_sum_y / 500.0f;    // proportional factor + convert from pixels to radians










 if(otoMeasure > targetOto-50){
    xOpt.compute(4,0,throttle,0,attitude[1],deltaX*100);
    yOpt.compute(5,0,throttle,0,attitude[0],deltaY*100);
  }
 
    //xOpt.Setp(0);
    //yOpt.Setp(0);
 //}
   
  Serial.print("X: ");
  Serial.print(deltaX);
  Serial.print(", Y: ");
  Serial.print(deltaY);
  Serial.print(", deltaCsr: ");
  Serial.print(deltaCsr);
  Serial.println();

}
}
#endif  
