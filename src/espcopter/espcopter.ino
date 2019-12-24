#include <ros.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/BatteryStatus.h>

#include <std_srvs/Trigger.h>

#include <ESP8266WiFi.h>
#include "espcopter.h"
//#include "remotexy.h"

// **************************************************


/* Wifi setup */
//IPAddress ROS_MASTER_ADDRESS(150, 164, 212, 125); // ros master ip
IPAddress ROS_MASTER_ADDRESS(192, 168, 2, 183); // ros master ip
char* WIFI_SSID = "Drone"; // network name
char* WIFI_PASSWD = "1234567890"; // network password

/* ROS Setup */
String drone_name;
/* ROS Node Instaciatation */
ros::NodeHandle nh;

/* LED callback */
void led_callback(const std_msgs::ColorRGBA& msg);
String led_topic;
ros::Subscriber<std_msgs::ColorRGBA> *led_sub;

/* Motor callback */
bool enable_motors_only = false;
int16_t pwmMotorFL_, pwmMotorFR_, pwmMotorRL_, pwmMotorRR_;
void motors_callback(const mavros_msgs::RCOut& msg);
String motors_topic;
ros::Subscriber<mavros_msgs::RCOut> *motors_sub;

/* Attitude callback */
void attitude_callback(const mavros_msgs::AttitudeTarget& msg);
String attitude_topic;
ros::Subscriber<mavros_msgs::AttitudeTarget> *attitude_sub;

/* Battery Status */
mavros_msgs::BatteryStatus battery_msg;     /* Message Type */
String battery_topic;                 /* Topic name */
ros::Publisher *battery_pub;          /* Publisher */
void update_battery(void);            /* Update Loop */

ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> *arm_motors_srv;         /* Service Type */
String arm_motors_topic;                                                                             /* Service name */
void arm_motors_callback(const std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> *calibration_srv;         /* Service Type */
String calibration_topic;                                                                             /* Service name */
void calibration_callback(const std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

char buf [200];

/* Timer */
double timer_ros, log_timer_ros, rate_ros = 50, timer_freq = 0, loop_freq = 0, control_loop = 0;

//int controlButton_1 = 0;
//int controlButton_2 = 0;

void setup() {
  /* Connect the ESP8266 the the wifi AP */
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  Serial.begin(115200);
  Serial.println("[ROS] Setting up...");

  /* Start ROS communication module */
  uint16_t ROS_MASTER_PORT = 11411;
  nh.getHardware()->setConnection(ROS_MASTER_ADDRESS, ROS_MASTER_PORT);

  /* Setup laser publisher */
  drone_name = String("/drone_1");

  led_topic = drone_name + String("/led");
  led_sub = new ros::Subscriber<std_msgs::ColorRGBA>(led_topic.c_str(), led_callback);

  motors_topic = drone_name + String("/motors");
  motors_sub = new ros::Subscriber<mavros_msgs::RCOut>(motors_topic.c_str(), motors_callback);

  attitude_topic = drone_name + String("/attitude");
  attitude_sub = new ros::Subscriber<mavros_msgs::AttitudeTarget>(attitude_topic.c_str(), attitude_callback);

  battery_topic = drone_name + String("/battery");                         /* Update topic name */
  battery_pub = new ros::Publisher(battery_topic.c_str(), &battery_msg);    /* Instantiate publisher */
  nh.advertise(*battery_pub); 

  arm_motors_topic = drone_name + String("/arm_motors");
  arm_motors_srv = new ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(arm_motors_topic.c_str(), &arm_motors_callback);
  nh.advertiseService(*arm_motors_srv);

  calibration_topic = drone_name + String("/calibration");
  calibration_srv = new ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(calibration_topic.c_str(), &calibration_callback);
  nh.advertiseService(*calibration_srv);

  /* Starting ros node */
  nh.initNode();

  /* Address Subscribers */
  nh.subscribe(*led_sub);
  nh.subscribe(*motors_sub);
  nh.subscribe(*attitude_sub);
  
  /* ROS LOG */
  sprintf(buf, "\33[96m---------------------\33[0m");
  nh.loginfo(buf);
  sprintf(buf, "\33[96mWelcome to ESPCopter!\33[0m");
  nh.loginfo(buf);
  sprintf(buf, "\33[96m---------------------\33[0m");
  nh.loginfo(buf);

  Serial.println("[ROS] Ready!");

  mainSetup();
  
  /*Trim_Roll = -125; // -1750, 1750
  Trim_Pitch = 250; // -1750, 1750
  Trim_Yaw = 250;  // -1750, 1750*/

  delay(1000);
}

void loop() {
  timer_freq = millis();
  /* ROS INFOS */
  if ((millis() - log_timer_ros) > 5000) {
    log_timer_ros = millis();
    sprintf(buf, "\33[96m[Drone-1] Conected at time: %d, loop_freq: %f, battery: %f\33[0m", millis(), loop_freq, (float)analogRead(A0) * 5.5);
    nh.loginfo(buf);
    sprintf(buf, "\33[96m[Drone-1] ArmControl: %d, landingoff: %d, stopFlightControl: %d\33[0m", armControl, landingOff, stopFlightControl);
    nh.loginfo(buf);
    sprintf(buf, "\33[96m[Drone-1] throttle: %f, motorFL: %f, motorFR: %f, motorRL: %f, motorRR: %f\33[0m", throttle, motorFL, motorFR, motorRL, motorRR);
    nh.loginfo(buf);
    
  }

  /* ROS Loop */
  if (millis() - timer_ros > rate_ros) {
    update_battery(); 
    timer_ros = millis();
    nh.spinOnce();
  }
  if (enable_motors_only){
      pwm_set_duty((pwmMotorFL_), 0);
      pwm_set_duty((pwmMotorFR_), 3);
      pwm_set_duty((pwmMotorRR_), 2);
      pwm_set_duty((pwmMotorRL_), 1);
      pwm_start();

  }else{
    //mainLoop();
    //modeControl();
      FlightControl(); 

  }
    
    
    loop_freq = 1000.0/(millis() - timer_freq);
    timer_freq = millis();
}

void led_callback(const std_msgs::ColorRGBA& msg) {
  analogWrite(redLed, (int)msg.r);
  analogWrite(greenLed, (int)msg.g);
  analogWrite(blueLed, (int)msg.b);
  sprintf(buf, "\33[96m[Drone-1] Turning the leds on RGB(%d, %d, %d)...\33[0m", (int)msg.r, (int)msg.g, (int)msg.b);
  nh.loginfo(buf);
}

void arm_motors_callback(const std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  armControl = !armControl;
  analogWriteFreq(20000);
  if (armControl == 1){
    sprintf(buf, "\33[91m[Drone-1] Be careful! Motors are enabled!\33[0m");
//    for (int i = 0; i < 12; i++) {
//      analogWrite(14, i * 10);
//      analogWrite(15, i * 10);
//      analogWrite(12, i * 10);
//      analogWrite(13, i * 10);
//      delay(25);
//    }
//    analogWrite(14, 0);
//    analogWrite(15, 0);
//    analogWrite(12, 0);
//    analogWrite(13, 0);
  } else {
    enable_motors_only = false;
    throttle = 0;
    sprintf(buf, "\33[92m[Drone-1] Motors are disabled!\33[0m");
  }  
  analogWriteFreq(20000);
  nh.loginfo(buf);
}

void motors_callback(const mavros_msgs::RCOut& msg){
  if (!armControl){
    sprintf(buf, "\33[93m[Drone-1] Motors are disarmed!\33[0m");
    nh.loginfo(buf);
    enable_motors_only = false;
    return;
  } 
  sprintf(buf, "\33[93m[Drone-1] Settings motors to: %d, %d, %d, %d \33[0m",msg.channels[0], msg.channels[1], msg.channels[2], msg.channels[3]);
  nh.loginfo(buf);
  
  pwmMotorFL_ = round(map(msg.channels[0], 0, 1023, 0, PWM_PERIOD)); //255
  pwmMotorFR_ = round(map(msg.channels[3], 0, 1023, 0, PWM_PERIOD));
  pwmMotorRL_ = round(map(msg.channels[2], 0, 1023, 0, PWM_PERIOD));
  pwmMotorRR_ = round(map(msg.channels[1], 0, 1023, 0, PWM_PERIOD));

  pwmMotorFL_ = constrain(pwmMotorFL_, 0, PWM_PERIOD);
  pwmMotorFR_ = constrain(pwmMotorFR_, 0, PWM_PERIOD);
  pwmMotorRL_ = constrain(pwmMotorRL_, 0, PWM_PERIOD);
  pwmMotorRR_ = constrain(pwmMotorRR_, 0, PWM_PERIOD);
  enable_motors_only = true;
}

void attitude_callback(const mavros_msgs::AttitudeTarget& msg){
  enable_motors_only = false;
  if (!armControl){
    sprintf(buf, "\33[93m[Drone-1] Motors are disarmed!\33[0m");
    nh.loginfo(buf);
    return;
  } 
  throttle = msg.thrust;
  if (throttle >= motorMax) {
    throttle = motorMax;
  } else if (throttle <= 0) {
    throttle = 0;
  }
  SetPoint[0] = map(msg.body_rate.x, -100, 100, -ROLL_LIMIT, ROLL_LIMIT);
  SetPoint[1] = map(msg.body_rate.y, -100, 100, -PITCH_LIMIT, PITCH_LIMIT);
  SetPoint[2] = map(msg.body_rate.z, -100, 100, -YAW_LIMIT, YAW_LIMIT);
}

void update_battery(void){
  battery_msg.voltage = analogRead(A0) * 5.5;
  battery_msg.remaining = battery_msg.voltage - 3000;
  battery_msg.header.stamp = nh.now();
  battery_pub->publish( &battery_msg );
}

void calibration_callback(const std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {  
  enable_motors_only = true;
  EEPROM.begin(512);
  sprintf(buf, "\33[93m[Drone-1] Starting calibration...\33[0m");
  nh.loginfo(buf);
  armControl = 0;
  sprintf(buf, "\33[92m[Drone-1] Motors are disabled!\33[0m");
  nh.loginfo(buf);

  //char *buf2 = ahrs.showParameter();
  
   ahrs.calibration();
   ahrs.loadParameter();
}



