
  int rudder_value = 0;
  //int x_axis = int(y);
  //int y_axis = int(-x);
  int action = 5;
int y_axis, x_axis;
void keyData() { 

    println(tgl20.getValue());
  switch(key) { 
    case('f'):throttleValue=0; y_axis = 0;x_axis = 0;rudder.setValue(0);rudder_value=0;break;
    case('9'):knobTurretRotation.setValue(0);throttleValue=0;break; // y_axis = 0;x_axis = 0;rudder.setValue(0);
    case('8'):if(throttleValue < 100){knobTurretRotation.setValue(throttleValue = throttleValue + 5);}break;
    case('5'):if(throttleValue > 0){knobTurretRotation.setValue(throttleValue = throttleValue - 5);}break;
    case'q':rudder.setValue(rudder_value = rudder_value - 2) ;break;
    case'e':rudder.setValue(rudder_value = rudder_value + 2) ;break;
    case'a': x_axis = x_axis + action ;break;
    case'd': x_axis = x_axis - action ;break;
    case'w':y_axis = y_axis - action  ;break;
    case's': y_axis = y_axis + action ;break;
    case'm': tgl20.setValue(false); isMouseTracking = false;break;
    case'n': tgl20.setValue(true); isMouseTracking = true;break;
    default:
    //println("Wrong key");   // Prints "Zulu"
    break;
   
  
    //case('2'):myKnobB.setConstrained(false).hideTickMarks().snapToTickMarks(false);break;
    //case('3'):myKnobA.shuffle();myKnobB.shuffle();break;
  }
   if(y_axis > 88){
     y_axis = 89;
   }
   if(y_axis < -88){
     y_axis = -89;
   }
    if(x_axis > 88){
     x_axis = 89;
   }
     if(x_axis < -88){
     x_axis = -89;
   }
}