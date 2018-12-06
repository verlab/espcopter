
long previousMillis = 0;
long interval = 200;


float[] output={0.0,0.0};
float[] output_={0.0,0.0};
boolean autoFly= false;
boolean autoOff = false;
float dt1;

float  timenow;
float  timeprev;

float[] P ={0.0,0.0};
float[] I ={0.0,0.0};
float[] D ={0.0,0.0};

float KPO = 0.1;
float KIO = 0.06;
float KDO = 0.08;

float KIO_;

int whichis;
/*

float KPO = 0.1;
float KIO = 0.05;
float KDO = 0.12;

float KPO = 0.1;
float KIO = 0.08;//100
float KDO = 0.1;

float KPO = 0.12;
float KIO = 0.012;
float KDO = 0.08;
*/
int limitUp = 30;
int limitDown = -30;
float[] motionFactor ={0.0,0.0};
float[] error ={0.0,0.0};
float[] error_ ={0.0,0.0};
float[] delta ={0.0,0.0};
float[] delta_ ={0.0,0.0};
float[] ITERM ={0.0,0.0};
float[] cordinate ={50,50};

int missionPoint = 1;
int rangeDrone =6;
long previousMillisDrone = 0;

int controlPid =0;
int missionPointCount=0;

int timerPID=0;

void pid(float xAxis_1, float yAxis_1, int whichis){
  
   text((int)round(output[1]) +","+ (int)round(output[0]), 770, 125+whichis*10); 
    
    text((int)round(cordinate[1]) +","+ (int)round(cordinate[0]), 770, 175+whichis*10); 
    

  if(autoFly==true ){
     long currentMillis = millis();
     float[] Axis_={yAxis_1 , xAxis_1};
  
  if( points.size() > missionPoint){
   cordinate[1] = map(points.get(missionPoint).getX(),0,640,0,100);
   cordinate[0] = map(points.get(missionPoint).getY(),0,480,0,100);
  }
    if(Axis_[0] - cordinate[0] < rangeDrone && Axis_[0] - cordinate[0]  > -rangeDrone && Axis_[1] - cordinate[1] < rangeDrone && Axis_[1] - cordinate[1]  > -rangeDrone ){
    
    }else{
     previousMillisDrone = currentMillis; 
    }
    
  
  // println("currentMillis " + currentMillis );


     if( points.size() > missionPoint){
       
       points.get(missionPoint).setCollor(1);
     if( (currentMillis - previousMillisDrone) > points.get(missionPoint).getTiming()  &&  missionPoint < points.size() ){
     if(missionPointCount ==0){
       points.get(missionPoint).setCollor(2);
        missionPoint++;
        missionPointCount =1;
      }       
      }else{
      missionPointCount =0;
      }
     }
     
//  print("missionPoint " + missionPoint ); 
  // print(" xaxis " + (Axis_[0] - cordinate[0]) ); 
   //println(" yaxis " + (Axis_[1] - cordinate[1]) ); 
  
   
 if(currentMillis - previousMillis >= interval  && autoFly==true ){
    previousMillis = currentMillis;  
    
    
    
    timenow = millis();
    dt1 = (timenow - timeprev)/1000;
    timeprev = timenow;
  
     for(int i = 0 ; i < 2 ; i++ ){
       
     error[i] = 10*(Axis_[i] - cordinate[i]);
         
   //  error[i] = constrain(error[i],-400,400);  
     
     P[i] = error[i] * KPO;
    
     ITERM[i] += error[i] * (float)dt1;
      
     ITERM[i] = constrain(ITERM[i], -10000, 10000);
     
     
    if(KIO != KIO_){
     ITERM[0]  = (ITERM[0]*KIO_)/KIO;
     ITERM[1]  = (ITERM[1]*KIO_)/KIO;
     }
     KIO_ = KIO;
       
     
     I[i] =  ITERM[i] * KIO;
     
      
     
     delta[i] = (error[i] - error_[i]) / (float)dt1;
     error_[i]  =  error[i]; 
        
     //delta[i] = 0.6 * (float)delta[i] + (1. - 0.6) * (float)delta_[i];

     //delta_[i] = delta[i];
  
     D[i] = KDO * delta[i];
       
      
     
     output[i] = P[i] + I[i]+ D[i];
     
     //output[i] = 0.75 * (float)output[i] + (1 - 0.75) * (float)output_[i];
    // output_[i] = output[i];
   
     /*
     if(controlPid == 0){
     output[i] = 0;
     P[i] =0;
     D[i] =0;
     I[i] =0;
     if(i ==1){
     controlPid = 1; 
     }
     }
     */
     /*
     if(controlPid == 1){
     output[i] = 0.95 * (float)output[i] + (1 - 0.95) * (float)output_[i];
     output_[i] = output[i];
     }*/
     
     if( i == 0){
     timerPID = timerPID+1;
     print("mission " + missionPoint );
     println(" I = " + I[i] +" D = " + D[i] +" P = "  + P[i] + " dt " + dt1);  
     }
    
     }
     /*
    if(timerPID > 150 ){
    if(Axis_[0] - cordinate[0] < rangeDrone && Axis_[0] - cordinate[0]  > -rangeDrone && Axis_[1] - cordinate[1] < rangeDrone && Axis_[1] - cordinate[1]  > -rangeDrone ){
    
    }else{
      controlPid = 1;
    /// KPO = 0.08;
    // KIO = 0.02;
   //  KDO = 0.06;
     print("NEW PID..............." );
     }
     }*/
    
   
    
    
   
    command =  (200+ (int)round(output[1])) + "," + (200 - (int)output[0] ) + "," + qx +","+ int((8*throttle)+round(Ticks3Value) )+ "." ;
   // println("from client: " + command);
    myClient.write(command);
    autoOff = true;
 }
 }else {
   for(int i = 0 ; i < 2 ; i++ ){
   motionFactor[i] = 0;
   error[i]= 0;
   error_[i]= 0;
   delta[i]= 0;
   ITERM[i]=0;
   output[i] = 0;
   }
   timerPID =0;
   controlPid =0;
   if(autoOff == true){
   command =  200+ "," + 200 + "," + qx +","+ int((8*throttle)+round(Ticks3Value) )+ "." ;
   println("from client: " + command);
   myClient.write(command);
   autoOff = false;
   }
 }


}

/*


       
       
  for(int i = 0 ; i < 2 ; i++ ){
    timenow[i] = millis();
     dt1[i] = (timenow[i] - timeprev[i])/1000;
    timeprev[i] = timenow[i];

    error[i] = 10*(Axis_[i]-50);
 
    ITERM[i] += error[i] * dt1[i];
     //text(ITERM[1], 770, 150); 
    // Calculate I-term
    ITERM[i] = constrain(ITERM[i], -900, 900);
    
    timenow2[i] = millis();
    dt2[i] = (float)(timenow2[i] - timeprev2[i])/1000;
    timeprev2[i] = timenow2[i];

    delta[i] = ((error[i] - error_[i])/dt2[i]);
    error_[i]  =  error[i]; 
    
    output[i] = error[i] * kp[i] + ki[i] * ITERM[i] - delta[i]*kd[i]; // Calculate P-term
    output[i] = constrain(output[i], -1000, 1000); 
  }
 */