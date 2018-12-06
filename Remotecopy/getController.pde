
int throttle =0;
int Ticks2Value = 0;
int Ticks2ValueX = 0;
float Ticks3Value = 0;
int Ticks3ValueX = 0;
int Ticks4Value = 0;
int Ticks4ValueX = 0;
int Ticks1Value = 0;
int Ticks1ValueX = 0;
float q = 0;
int qx = 0;

int motorPower;
String command;
int XAxis;
int YAxis;
int boundRate;

int timing;

void getController(){
  boundRate = round(cp5.getController("BoundRate").getValue());
  
  throttle = round(cp5.getController("Motor Power").getValue());
 
  Ticks4Value =  round(cp5.getController("sliderTicks4").getValue());
  Ticks3Value = cp5.getController("sliderTicks3").getValue();

  Ticks2Value = round(cp5.getController("sliderTicks1").getValue());
  Ticks1Value = round(cp5.getController("sliderTicks2").getValue());
  
  Ticks1Value = round(map(Ticks1Value,-100,100, 0,200));
  Ticks2Value = round(map(Ticks2Value,-100,100, 0,200));
  Ticks4Value = round(map(Ticks4Value,-100,100, 0,200));

  q = cp5.getController("slider").getValue();
qx = round(q) +200 +12*rudder_jstc;

   /*
  XAxis = int(y- (x_axis*2));
  YAxis =  int(x- (y_axis*2) );
  */
  XAxis = int((20*round(x)) - (x_axis*2));
  YAxis =  int((20*round(y))- (y_axis*2) );
  
   // XAxis = XAxis - 2*round(Ticks2Value) ;
   // YAxis = YAxis + 2*round(Ticks1Value) ;
   
 
   XAxis= round(map(XAxis, -200, 200, 100,300)) - 10*elevator_jstc;
   YAxis= round(map(YAxis, -200, 200, 100,300)) + 10*aileron_jstc;
   //qx= 200+12*rudder_jstc ;
      
  
   if(ifConsole){
     if(throttle_jstc != throttle_jstc_old){
     knobTurretRotation.setValue(throttle_jstc);
     }
     if(R2On &&  motorPower < 100 ){
      motorPower = motorPower+ 3;
      knobTurretRotation.setValue(motorPower);
        }
        
      if(R1On && motorPower > 0){
      motorPower = motorPower- 3;
      knobTurretRotation.setValue(motorPower);
        }
      if(L1On &&  motorPower < 30 ){
     rudder.setValue(rudder_value = rudder_value + 2);
        }
        
      if(L2On && motorPower > -30){
  rudder.setValue(rudder_value = rudder_value - 2);
        }
        
        
        
        
   }
      
    
  
 
 command =  YAxis  + "," + XAxis + "," + qx +","+ int((7.5*throttle)+round(Ticks3Value) )+ "." ;
 if ((mouseButton == LEFT) &&  clorWheelweight < mouseX && clorWheelweight+201 > mouseX && clorWheelheight < mouseY && clorWheelheight+201 > mouseY ){ // wheelclor
   command =  cp5.get(ColorWheel.class,"colorWheel").b()*4 + "*" + cp5.get(ColorWheel.class,"colorWheel").g()*4 + "*" +cp5.get(ColorWheel.class,"colorWheel").r()*4 + "?" ;
 }
 
}
/*
void mode(boolean theFlag) {
  
   if( 0 == cp5.getController("mode").getValue()) {
   command = "a";
     
   }else if( 1 == cp5.getController("mode").getValue()){
   command = "b";
     
   }else if( 2 == cp5.getController("mode").getValue()){
      command = "c";
     
   }else if( 3 == cp5.getController("mode").getValue()){
      command = "d";
     
   }else if( 4 == cp5.getController("mode").getValue()){
      command = "e";
     
   }else if( 5 == cp5.getController("mode").getValue()){
      command = "f";
     
   }
   
   myClient.write(command);
   println("from client: " +  command);
   delay(100);
}*/


void Calibration(boolean theFlag) {
  command = "C";
  myClient.write(command);
  println("from client: " + command);
}

void ARM__DISARM(boolean theFlag) {
  if(theFlag==true) {
    command = "Q";
  } else {
    command = "q";
  }
  delay(250);
  myClient.write(command);
  println("a Data event." +  command);
}

void Mode_1(boolean theFlag) {
  if(theFlag==true) {
    command = "A";
  } else {
    command = "a";
  }
  delay(250);
  myClient.write(command);
  println("a Data event." +  command);
}
void Mode_2(boolean theFlag) {
  if(theFlag==true) {
    command = "B";
  } else {
    command = "b";
  }
  delay(250);
  myClient.write(command);
  println("a Data event." +  command);
}
void Mode_3(boolean theFlag) {
  if(theFlag==true) {
    command = "C";
  } else {
    command = "c";
  }
  delay(250);
  myClient.write(command);
  println("a Data event." +  command);
}



void Take__Data(boolean theFlag) {
  if(theFlag==true) {
    command = "W";
  } else {
    command = "w";
  }
  myClient.write(command);
  println("a Data event." +  command);
}
void tglAuto(boolean theFlag) {
  if(theFlag==true) {
     command = "q";
     myClient.write(command);
     delay(100);
    command = "c";
     myClient.write(command);
     delay(100);
     command = "Q";
     myClient.write(command);
     delay(100);
     println("a Data event." +  command);
       autoFly = true;
  } else {
  command = "q";
  autoFly = false;
  myClient.write(command);
  println("a Data event." +  command);
  myClient.write("200,200,200,0.");
  } 
}
void Time(int theFlag) {
  timing = theFlag;
  println("Time " + theFlag);
}



void Connect(boolean theFlag) {
 
  if(theFlag==true) {
     connectDrone();
     } else { 

       
   }    
  }