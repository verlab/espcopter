//---------------------------------------------------------------------------------------
//Define PWM pin

int pinPWM[4] = {14, 15, 12, 13}; 

//---------------------------------------------------------------------------------------
//Define Led pin
#define blueLed 16 
#define redLed 2
#define greenLed 0
//---------------------------------------------------------------------------------------
// Define for remote method

//#define REMOTE_XY_REMOTE

//#define PROCESSING_REMOTE
//#define PROCESSING_SINGLE
//#define PROCESSING_MULTIPLE

//#define PPM_REMOTE

#define REMOTE_WEB_APP // Phone control

//---------------------------------------------------------------------------------------
//Define modules

//#define NeoPixel

//#define MULTI_RANGER

#define vl53l0x // auto recognazation 
#define LONG_RANGE 
//#define HIGH_ACCURACY 
//#define HIGH_SPEED

//#define opticalFlow
//---------------------------------------------------------------------------------------

// Define trim and Set point values 
int SetPoint[3] = {0}, _SetPoint[3] = {0}; // -1750 : 1750
int Trim_Roll = 0, Trim_Pitch = 0; // -1750 : 1750
int Trim_Yaw =0; // -8000 : 8000
float throttle = 0;
// Define auto altitude (mm)
int targetOto = 250;
//---------------------------------------------------------------------------------------
// Define fly modes 

int flyMode_1 = 0, flyMode_2 = 0 , flyMode_3 = 0;

//---------------------------------------------------------------------------------------
// Define controls

int firstStage= 0; 
int armControl = 0;
int batteryControl = 1;
int batteryCount = 0;
int vl5310xControl = 0;
int throttleControl  = 0;
int yawControl  = 0;
int stopFlightControl  = 1;

//---------------------------------------------------------------------------------------
// Define RX Value

float RX_throttle = 0; // 0 : 700
float RX_roll = 0, RX_pitch = 0, RX_yaw = 0;  // -100 : + 100


//---------------------------------------------------------------------------------------
// Define other variables


float magOffsetMotorFnl[3] ={0};
float magOffsetMotorCount =0;
float magOffsetMotor[3] = {0};
float sumMagOffsetMotor[3] = {0};
float magOffsetMotorCount2 =0;
float magOffsetMotor2[3] = {0};
float sumMagOffsetMotor2[3] = {0};
  
unsigned long calMillis = 0;        // will store last time LED was updated

// constants won't change:
const long calInterval = 1000; 

float errorDegrees = 0;
int motorMax = 700;

#define weightOfDrone  1

float xOptical = 0;
float yOptical = 0;
float crcOpt = 0;
float crcOptQuality  = 0;

String inString = "";    // string to hold input
int currentCommand = 0;
int currentColor = 0;
int currentPid = 0;
int currentValue = 0;

int  boundRate = 1000;
double aggKi;
double aggKp;

boolean otoHover = false;
boolean sendMpu = false;

int calAHRS = 0;
int calTest = 1;


char getMode = 0;
int i;

float degree[4] = {0};


float altitudeBmp = 0;
float temperatureBmp = 0;

float  rate[3] = {0};
float  attitude_rate[3] = {0};
double attitude[3] = {0};
double attitude_Radian[3] = {0};
int  _Rate[3] = {0};
int16_t  _Angle[3] = {0};
long ctr_value[3];


float Alpha = 0.75, a = 0.7585; //Trim_Roll = -150, Trim_Pitch = 100, 

int dt, time_prev = 0, time_now, dt2, time_prev2 = 0;

float yaw_rate = 0, motorFL = 0, motorFR = 0, motorRL = 0, motorRR = 0;
