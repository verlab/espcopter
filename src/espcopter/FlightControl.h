#include "Parameter.h"

unsigned long ts = millis();

unsigned long tsbat = millis();
boolean ledState =   HIGH;

float factor;
float factor_;

int stopFlightHand = 0;
void FlightControl() {

  if ((millis() - tsbat) > 1000) { //Update only on
    tsbat = millis();
    //Serial.println(analogRead(A0)*5.5);
    if (analogRead(A0) * 5.5  < 3000) {
      batteryCount = batteryCount + 1;

    } else {
      batteryCount = 0;
    }
    if (batteryCount > 25) {
      //landingOff = 0;
    }
  }

  if ((millis() - ts) > 2) { //Update only once per 2ms (500hz update rate)
    ts = millis();
    ahrs.compute(attitude, rate, attitudeRadian, rateRadian);

    ahrs.headingMag(rateRadian, attitudeRadian, degree , throttle);

    //yawControl = 0;
    //if ( yawControl == 0) {
    //  degree[0] = 0;
    //}

    if (throttle > 2) {
      roll.compute( 0 , SetPoint[0], throttle, Trim_Roll + Trim_Roll_Bs, -attitude[1], rate[1]); // Trim_Roll xOpt.output
      pitch.compute( 1 , SetPoint[1], throttle, Trim_Pitch +  Trim_Pitch_Bs, -attitude[0], -rate[0]);
      yaw.compute( 2 , SetPoint[2], throttle, Trim_Yaw, (degree[0]) * 100, -rate[2]); //-rate[2]degree[0]*100
      // roll.compute( 0 , SetPoint[0] + SetPointOptMain[0], throttle, -xGOT.output - xOpt.output + Trim_Roll + Trim_Roll_Bs - xMulti.output + xMultiM.output  , -attitude[1], rate[1]); // Trim_Roll xOpt.output
      // pitch.compute( 1 , SetPoint[1] + SetPointOptMain[1] , throttle, -yGOT.output - yOpt.output +  Trim_Pitch +  Trim_Pitch_Bs - yMulti.output  + yMultiM.output  , -attitude[0], -rate[0]);
      // yaw.compute( 2 , SetPoint[2] , throttle, Trim_Yaw, (degree[0] + yaw_Control) * 100, -rate[2]); //-rate[2]degree[0]*100
    } else {
      ahrs.setZero();
    }
  }

  factor = 1  +  throttle / 85.0; //82 //+pow(2,throttle/225);// 1 + pow(2,throttle/225);
  factor_ = 100;
  
  motorFL = throttle + (roll.output / factor_) * factor - (pitch.output / factor_) * factor + (yaw.output / factor_) * factor ; //+ (xOpt.GetItermRateBase()/25);// + (yOpt.GetItermRateBase()/25); // pitch
  motorFR = throttle - (roll.output / factor_) * factor - (pitch.output / factor_) * factor - (yaw.output / factor_) * factor ; //- (xOpt.GetItermRateBase()/25)+100;// + (yOpt.GetItermRateBase()/25); //rool
  motorRL = throttle + (roll.output / factor_) * factor + (pitch.output / factor_) * factor - (yaw.output / factor_) * factor ; //+ (xOpt.GetItermRateBase()/25);// - (yOpt.GetItermRateBase()/25); // pitch
  motorRR = throttle - (roll.output / factor_) * factor + (pitch.output / factor_) * factor + (yaw.output / factor_) * factor ; //- (xOpt.GetItermRateBase()/25)+100;// - (yOpt.GetItermRateBase()/25); //rool

  int16_t motorFL_ = round(map(motorFL, 0, 1023, 0, PWM_PERIOD)); //255
  int16_t motorFR_ = round(map(motorFR, 0, 1023, 0, PWM_PERIOD));
  int16_t motorRL_ = round(map(motorRL, 0, 1023, 0, PWM_PERIOD));
  int16_t motorRR_ = round(map(motorRR, 0, 1023, 0, PWM_PERIOD));

  motorFL_ = constrain(motorFL_, 0, PWM_PERIOD);
  motorFR_ = constrain(motorFR_, 0, PWM_PERIOD);
  motorRL_ = constrain(motorRL_, 0, PWM_PERIOD);
  motorRR_ = constrain(motorRR_, 0, PWM_PERIOD);

  // Input Control Value to PWM Pins
  if (throttle > 2) {
    pwm_set_duty((motorFL_), 0);
    pwm_set_duty((motorFR_), 3);
    pwm_set_duty((motorRR_), 2);
    pwm_set_duty((motorRL_), 1);
  } else {
    pwm_set_duty(0, 0);
    pwm_set_duty(0, 3);
    pwm_set_duty(0, 2);
    pwm_set_duty(0, 1);
  }
  pwm_start();

}

//
//void modeControl() {
//  if (armControl == 1 && landingOff == 1 && stopFlightControl == 1) {
//
//    throttleControl = 1;
//    if (flyMode_1 == 1) {
//      yawControl = 1;
//    } else {
//      yawControl = 0;
//    }
//
//    if (flyMode_2 == 1) {
//      throttleControl = 0;
//      otoHover = 1;
//    } else {
//      otoHover = 0;
//    }
//
//    if (flyMode_3 == 1) {
//      throttleControl = 0;
//      autoOpt = 1;
//      otoHover = 1;
//    } else if ( flyMode_2 != 1) {
//      autoOpt = 0;
//      otoHover = 0;
//      //throttleControl = 0;
//    } else {
//      autoOpt = 0;
//      //throttleControl = 0;
//    }
//
//  } else if (landingOff == 0) {
//    //throttleControl = 0;
//    //otoHover=0;
//    //autoOpt =0;
//    //yawControl=0;
//    targetOto = 1;
//    if ( otoMeasure < 200) {
//      throttleControl = 0;
//      throttle = throttle - 2;
//      yawControl = 0;
//      otoHover = 0;
//      autoOpt = 0;
//      if (throttle < 200) {
//        throttle = 0;
//        landingOff = 1;
//        targetOto = targetOtoDf;
//        flyMode_1 = 0;
//        flyMode_2 = 0;
//        flyMode_3 = 0;
//        missionCounter = -1;
//        takeMissions = 1;
//      }
//    }
//
//
//  } else if (stopFlightControl == 0) {
//    armControl = 0;
//    throttleControl = 0;
//    throttle = 0;
//    otoHover = 0;
//    autoOpt = 0;
//    yawControl = 0;
//  } else {
//    throttleControl = 0;
//    otoHover = 0;
//    autoOpt = 0;
//    yawControl = 0;
//    throttle = 0;
//
//    missionCounter = -1;
//    takeMissions = 1;
//
//  }
//  /*
//      if(otoHover == 0 && throttleControl == 0  ){
//      throttle = throttle - 0.1;
//      if( throttle < 250){
//      throttle=0;
//      }
//      //throttleControl = 1;
//      }*/
//
//  if (throttleControl == 1 ) {
//    //throttle= RX_throttle ;
//    if ( throttle < RX_throttle) {
//      throttle = throttle + 2.5;
//    }
//    if ( throttle > RX_throttle) {
//      throttle = throttle - 2.5;
//    }
//  } else {
//
//
//    // targetOto = constrain(RX_throttle,250,500);
//  }
//
//
//  SetPoint[0] = map(RX_roll, -100, 100, -ROLL_LIMIT, ROLL_LIMIT);
//  SetPoint[1] = map(RX_pitch, -100, 100, -PITCH_LIMIT, PITCH_LIMIT);
//  /*
//     #if defined(MQTT) == 0
//     if(opticalFlowControl == 1 && autoOpt == 1){
//     SetPointOpt[0] = -map(RX_roll, -100, 100, -15, 15);
//     SetPointOpt[1] = -map(RX_pitch, -100, 100, -15, 15);
//     SetPoint[0] =0;
//     SetPoint[1] =0;
//     }
//     #endif*/
//
//  if (yawControl == 0 ) {
//    SetPoint[2] = map(RX_yaw, -100, 100, -YAW_LIMIT, YAW_LIMIT);
//  } else {
//    SetPoint[2] = 0;
//  }
//
//  throttle = constrain(throttle, 0, 700);
//
//}



float getMpuAttitudeX() {
  return attitude[0];
}

float getMpuAttitudeY() {
  return attitude[1];
}
float getMpuAttitudeZ() {
  return attitude[2];
}

float getMpuRateX() {
  return rate[0];
}

float getMpuRateY() {
  return rate[1];
}
float getMpuTemp() {
  return ((float) ahrs.readTemp()) / 333.87f + 21.0f;
}

void setTrimRoll(int x) {
  Trim_Roll = x;
}

void setTrimPitch(int x) {
  Trim_Pitch = x;
}
void setTrimYaw(int x) {
  Trim_Yaw = x;
  //yaw.SetItermRate(x);
}


void setArmControl(boolean x) {
  if ( x == true) {
    armControl = 1;
  } else {
    armControl = 0;
  }
}

void setFlyMode_1(boolean x) {
  if ( x == true) {
    flyMode_1 = 1;
  } else {
    flyMode_1 = 0;
  }
}


void setFlyMode_2(boolean x) {
  if ( x == true) {
    flyMode_2 = 1;
  } else {
    flyMode_2 = 0;
  }
}

void setFlyMode_3(boolean x) {
  if ( x == true) {
    flyMode_3 = 1;
  } else {
    flyMode_3 = 0;
  }
}

void landing(boolean x) {
  if ( x == true) {
    landingOff = 0;
  } else {
    landingOff = 1;
  }
}

void setMotorMax(int x) {
  motorMax = x;
}

int getRX_throttle() {
  return round(RX_throttle);
}

int getRX_roll() {
  return round(RX_throttle);
}

int getRX_pitch() {
  return round(RX_pitch);
}

int getRX_yaw() {
  return round(RX_yaw);
}


//void setTargetOto(int x) {
//  targetOto = x;
//}
//
//int getOtoMeasure() {
//  return otoMeasure;
//}


/*void SetOptPoint_X(int x) {
  SetPointOpt[0] = x;
  }
  void SetOptPoint_Y(int x) {
  SetPointOpt[1] = x;
  }

  int getOptData_X() {
  return deltaCalX;
  }

  int getOptData_Y() {
  return deltaCalY;
  }
*/
