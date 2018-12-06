import controlP5.*;
import processing.net.*; 
import java.awt.Robot;
import java.awt.Point;
import java.awt.MouseInfo;

Robot rbt;

Client myClient; 
Client myClient_1; 
Client myClient_2; 
Client myClient_3; 

int XAxisBase;
int YAxisBase;
int qxBase;
int sxBase;
int Ticks2ValueBase;
int Ticks1ValueBase;
int Ticks4ValueBase;
int boundRateBase;
int bn = 1;
String textValue = "";
boolean isMoving = false; 
int throttleValue  = 0;
boolean ifConsole= false;
boolean ifVideo= false;
boolean ifMission= true;
String missionName= "missionBase.txt";

int droneNum = 1;

PrintWriter outputPs;


   void settings() {
      size(1700, 800,P3D);
  }


void setup() {
   background(0);
   
   connectDrone();
   

  if(ifConsole)consoleinit();
  if(ifVideo)opencCvinit();
   setupUI();
   setupJoystick(220+xDegree, 575+yDegree);

   
   try {
    rbt = new Robot();
  } catch(Exception e) {
    e.printStackTrace();
  }

  
}
// Default Response
String lastCommand = "M,0,0";
String response = "";
boolean start = false;
 void draw() {
getSurface().setLocation(100,100);
   startup();
   surface();
   if(ifConsole)getUserInput();
   if(ifVideo)openCvvideo();
    autonomus();  
    if( (boundRateBase != boundRate)  ){
    boundRateBase = boundRate;
    command = 0 + "!" +  0 + "!"+ boundRate + "&"; 
    println("from client: " + command);
    myClient.write(command);
    } 
   int errorControl=5;
    if( (XAxisBase-errorControl > XAxis) || (XAxisBase+errorControl < XAxis) || (YAxisBase-errorControl >  YAxis) || (YAxisBase+errorControl <  YAxis) || (qxBase !=  qx) || (throttle !=  sxBase)  ){ //&& autoFly==false
    println("from client: " + command);
    myClient.write(command);
    XAxisBase = XAxis;
    YAxisBase = YAxis;
    qxBase = qx;
    sxBase = throttle ;
    } 
  
    if( (Ticks1ValueBase != Ticks1Value) ||  (Ticks2ValueBase !=  Ticks2Value) ||(Ticks4ValueBase !=  Ticks4Value)){
    Ticks2ValueBase = Ticks2Value;
    Ticks1ValueBase = Ticks1Value;
    Ticks4ValueBase = Ticks4Value;
    command = Ticks1ValueBase + "+" +  Ticks2ValueBase + "+"+ Ticks4ValueBase + "-"; 
    println("from client: " + command);
    myClient.write(command);
    } 
  
  
   if(keyPressed){
   keyData(); 
   }
  joystickDraw();
  myClientAvaible();
  getController();   
 }
 
 void connectDrone(){
 
  myClient = new Client(this, "192.168.4.1", 80); 
   println(myClient.ip());
 }
 
 