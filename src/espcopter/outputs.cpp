#include "outputs.h"

void ESPCOPTER::espcopterSetup() {
  pinMode(blueLed_, OUTPUT);
  pinMode(redLed_, OUTPUT);
  pinMode(greenLed_, OUTPUT);

  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);

  redLed_Digital(0);
  blueLed_Digital(0);
  greenLed_Digital(0);

  motorFL_Analog(0);
  motorFR_Analog(0);
  motorRL_Analog(0);
  motorRR_Analog(0);

}

void ESPCOPTER::redLed_Digital(int OnOff) {
  OnOff = constrain(OnOff, 0, 1);
  if (OnOff == 1) {
    OnOff = 0;
  } else if (OnOff == 0) {
    OnOff = 1;
  }
  digitalWrite(redLed_, OnOff);
}

void ESPCOPTER::blueLed_Digital(int OnOff) {
  OnOff = constrain(OnOff, 0, 1);
  digitalWrite(blueLed_, OnOff);
}

void ESPCOPTER::greenLed_Digital(int OnOff) {
  OnOff = constrain(OnOff, 0, 1);
  if (OnOff == 1) {
    OnOff = 0;
  } else if (OnOff == 0) {
    OnOff = 1;
  }
  digitalWrite(greenLed_, OnOff);
}

void ESPCOPTER::redLed_Digital(boolean OnOff) {
  if (OnOff == true) {
    OnOff = false;
  } else if (OnOff == false) {
    OnOff = true;
  }
  digitalWrite(blueLed_, OnOff);
}

void ESPCOPTER::blueLed_Digital(boolean OnOff) {
  digitalWrite(blueLed_, OnOff);
}

void ESPCOPTER::greenLed_Digital(boolean OnOff) {
  if (OnOff == true) {
    OnOff = false;
  } else if (OnOff == false) {
    OnOff = true;
  }
  digitalWrite(greenLed_, OnOff);
}

/*
  void ESPCOPTER::redLed_Analog(int value){
   value= constrain(value, 0,1023);
   value = map(value,0,1023,1023,0);
   analogWrite(redLed_, value);
  }
  void ESPCOPTER::blueLed_Analog(int value){
   value= constrain(value, 0,1023);
   analogWrite(blueLed_, value);
  }
  void ESPCOPTER::greenLed_Analog(int value){
   value= constrain(value, 0,1023);
   value = map(value,0,1023,1023,0);
   analogWrite(greenLed_, value);
  }*/
void ESPCOPTER::motorFL_Analog(int value) {
  value = constrain(value, 0, 200);
  analogWrite(16, value);
}

void ESPCOPTER::motorFR_Analog(int value) {
  value = constrain(value, 0, 200);
  analogWrite(14, value);
}

void ESPCOPTER::motorRL_Analog(int value) {
  value = constrain(value, 0, 200);
  analogWrite(12, value);
}

void ESPCOPTER::motorRR_Analog(int value) {
  value = constrain(value, 0, 200);
  analogWrite(16, value);
}

int mettee  = 0;

void ESPCOPTER::buzzer(boolean OnOff) {
  unsigned long currentMillisBuzzer = millis();
  // if (currentMillisBuzzer - previousMillisBuzzer >= 10) {
  previousMillisBuzzer = currentMillisBuzzer;

  if (mettee == 0) {
    mettee = 1000;
  } else {
    mettee = 0;
  }
  analogWrite(2, mettee);
  // }

}
