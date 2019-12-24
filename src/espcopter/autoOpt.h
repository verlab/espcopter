int missionCounter = -1;
int takeMissions = 1;
int altitudeAuto[25]; 
int rollAuto[25]; 
int pitcthAuto[25]; 
int timeAuto[25]; 
int missionNumber =1;

int optFlyControl =0;

unsigned long currentTimingOpt;
unsigned long previousTimingOpt;
unsigned long GeneralTimingOpt;

void takeOff(int altitude, int delay_){
if(takeMissions == 1){
if( missionCounter == -1){
//batteryControl =1;  
flyMode_3 = 1;
targetOto = altitude;
}
GeneralTimingOpt = millis();
previousTimingOpt = millis();
missionCounter++;
altitudeAuto[missionCounter] = altitude;
rollAuto[missionCounter] = 0;
pitcthAuto[missionCounter] = 0;
delay_ = constrain(delay_,3000, 300000);
timeAuto[missionCounter]=delay_;
  }
}
void goForward(int delay_){
if(takeMissions == 1){
  missionCounter++;
  altitudeAuto[missionCounter] = altitudeAuto[missionCounter-1];
  rollAuto[missionCounter] = 0;
  pitcthAuto[missionCounter] = -15;
  timeAuto[missionCounter]=delay_;
  }
}
void goRight(int delay_){
if(takeMissions == 1){
  missionCounter++;
  altitudeAuto[missionCounter] = altitudeAuto[missionCounter-1];
  rollAuto[missionCounter] = -15;
  pitcthAuto[missionCounter] = 0;
  timeAuto[missionCounter]=delay_;
  }
}
void goLeft(int delay_){
if(takeMissions == 1){
  missionCounter++;
  altitudeAuto[missionCounter] = altitudeAuto[missionCounter-1];
  rollAuto[missionCounter] = 15;
  pitcthAuto[missionCounter] = 0;
  timeAuto[missionCounter]=delay_;
  }
}
void goBack(int delay_){
if(takeMissions == 1){
  missionCounter++;
  altitudeAuto[missionCounter] = altitudeAuto[missionCounter-1];
  rollAuto[missionCounter] = 0;
  pitcthAuto[missionCounter] = 20;
  timeAuto[missionCounter]=delay_;
  }
}
void delay_(int delay_){
  if(takeMissions == 1){
  missionCounter++;
  altitudeAuto[missionCounter] = altitudeAuto[missionCounter-1];
  rollAuto[missionCounter] = 0;
  pitcthAuto[missionCounter] = 0;
  timeAuto[missionCounter]=delay_;
  }
}
  void setAltitude(int altitude){
if(takeMissions == 1){
  missionCounter++;
  altitudeAuto[missionCounter] = altitude;
  rollAuto[missionCounter] = 0;
  pitcthAuto[missionCounter] = 0;
  timeAuto[missionCounter]=0;
  }
}

void land(){
//missionCounter = -1;
takeMissions = 0;
optFlyControl = optFlyControl + 1;
//batteryControl = 0;
}

void fly(){
  if(optFlyControl > 0 && armControl == 1){
 currentTimingOpt = millis();
    
    if(timeAuto[missionNumber] != 0){
    SetPointOpt[0]= rollAuto[missionNumber];
    SetPointOpt[1]= pitcthAuto[missionNumber];
    targetOto = altitudeAuto[missionCounter];
   
    if(currentTimingOpt - previousTimingOpt > timeAuto[missionNumber] ){    
    previousTimingOpt =  currentTimingOpt;
    SetPointOpt[0]= 0;
    SetPointOpt[1]= 0;
    targetOto = altitudeAuto[missionCounter];
    missionNumber++;
    }}
  
  

  if(currentTimingOpt - GeneralTimingOpt > timeAuto[0] ){
   landingOff = 0;
   //otoLanding();
  }

  }
}
