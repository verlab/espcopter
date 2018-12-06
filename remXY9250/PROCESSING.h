#ifdef PROCESSING_REMOTE
int red, green, blue = 0;

int  xAxis= 200, yAxis= 200, zAxis = 200 , trimX = 100, trimY = 100, trimZ =100;

unsigned long previousMillisTrail = 0;  
String strial;

#include <ESP8266WiFi.h>

const char WiFiAPPSK[] = "";
WiFiServer server(80);




#ifdef PROCESSING_SINGLE
void setupWiFi()
{
  WiFi.mode(WIFI_AP);

  // Do a little work to get a unique-ish name. Append the
  // last two bytes of the MAC (HEX'd) to "Thing-":
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();
  String AP_NameString = "ESPCopter";

  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);

  for (int i=0; i<AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);

   WiFi.softAP(AP_NameChar, WiFiAPPSK);

   server.begin();
}
#endif 

#ifdef PROCESSING_MULTIPLE

const char* ssid     = "ESPcopter-C";
const char* password = "123456789";

void setupWiFi(){
  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  
  // Start the server
  server.begin();

  Serial.println("Server started");
  IPAddress ip(192, 168, 137, 126); //192.168.137.126
  IPAddress gateway(ip[0], ip[1], ip[2], 1); // set gatway to ... 1
  IPAddress subnet(255, 255, 255, 0);
  Serial.print(F("Setting ip to: "));
  Serial.println(ip);
  Serial.print(F("Setting gateway to: "));
  Serial.println(gateway);
  Serial.print(F("Setting subnet to: "));
  Serial.println(subnet);
  if (!WiFi.config(ip, gateway, subnet, gateway)) {
    Serial.println("WiFi.config() failed");
  }
  Serial.println("WiFi.config() succeeded");
  
  // Print the IP address
  Serial.println(WiFi.localIP());
}
#endif 


void getRX(){

WiFiClient client = server.available();
// Serial.println("wait ");
 // Serial.println(client);
  if (client ) {

    while (client.connected()) {
      while(1){
        
      if (client.available()) {
   
        int inChar;
        inChar = client.read();
        Serial.println(inChar);
         switch (inChar) {
      case 'A':
      flyMode_1 = 1;
        break;
      case 'a':
     flyMode_1 = 0;
        break;
      case 'B':
      flyMode_2 = 1;
      break;
      case 'b':
      flyMode_2 = 0;
      break;
      case 'C':
      flyMode_3 = 1;
      break;
      case 'c':
       flyMode_3 = 0;
      break;
      case 'Q':
      armControl = 1;
      break;
      case 'q':
      armControl = 0;
      break;
      case 'W':
      sendMpu = true;
      break;
      case 'w':
      sendMpu = false;
      break;
      }
     if (isDigit(inChar)) {
    // convert the incoming byte to a char 
    // and add it to the string:
    inString += (char)inChar; 
  }

   // if you get a comma, convert to a number,
  // set the appropriate color, and increment
  // the color counter:
  if (inChar == ',') {
    // do something different for each value of currentColor:
    switch (currentCommand) {
      case 0:    // 0 = red
        xAxis = inString.toInt();
        // clear the string for new input:
        inString = "";
        break;
      case 1:    // 1 = green:
        yAxis = inString.toInt();
        // clear the string for new input:
        inString = "";
        break;
      case 2:    // 1 = green:
        zAxis = inString.toInt();
        // clear the string for new input:
        inString = "";
        break;
        
    }
    currentCommand++;
  }
   // if you get a newline, you know you've got
  // the last color, i.e. blue:
  if (inChar == '.') {
      if( otoHover == false ){//false
    RX_throttle = inString.toInt();
      }
    // you're raising the level on the anode:

    // clear the string for new input:
    inString = "";
    // reset the color counter:
    currentCommand = 0;
     }
  if (inChar == '+') {
    // do something different for each value of currentColor:
    switch (currentPid) {
      case 0:    // 0 = red
        trimX = inString.toInt();
        // clear the string for new input:
        inString = "";
        break;
      case 1: // 1 = green:
        trimY = inString.toInt();
        // clear the string for new input:
        inString = "";
        break;    
        }
    currentPid++;
  }
   // if you get a newline, you know you've got
  // the last color, i.e. blue:
  if (inChar == '-') {
     trimZ = inString.toInt();
    // you're raising the level on the anode:
    
    // clear the string for new input:
    inString = "";
    // reset the color counter:
    currentPid = 0;
     }

     

      // if you get a comma, convert to a number,
  // set the appropriate color, and increment
  // the color counter:
  if (inChar == '*') {
    // do something different for each value of currentColor:
    switch (currentColor) {
      case 0:    // 0 = red
        red = inString.toInt();
        // clear the string for new input:
        inString = "";
        break;
      case 1:    // 1 = green:
        green = inString.toInt();
        // clear the string for new input:
        inString = "";
        break;
    }
    currentColor++;
  }
  // if you get a newline, you know you've got
  // the last color, i.e. blue:
  if (inChar == '?') {
    blue = inString.toInt();
    // clear the string for new input:
    inString = "";
    // reset the color counter:
    currentColor = 0;
  }
    if (inChar == '!') {
    // do something different for each value of currentColor:
    switch (currentColor) {
      case 0:    // 0 = red
        red = inString.toInt();
        // clear the string for new input:
        inString = "";
        break;
      case 1:    // 1 = green:
        green = inString.toInt();
        // clear the string for new input:
        inString = "";
        break;
    }
    currentValue++;
  }
  // if you get a newline, you know you've got
  // the last color, i.e. blue:
  if (inChar == '&') {
    blue = inString.toInt();
    // clear the string for new input:
    inString = "";
    // reset the color counter:
    currentValue = 0;
  }
    }

  RX_roll = map(xAxis, 0, 400, -100, 100);
  RX_pitch = map(yAxis, 0, 400, -100, 100);
  RX_yaw = map(zAxis, 0, 400, -100, 100);  

  Trim_Roll = map(trimX, 0, 200, -ROLL_LIMIT, ROLL_LIMIT);
  Trim_Pitch = map(trimY, 0, 200, -PITCH_LIMIT, PITCH_LIMIT);
  Trim_Yaw = map(trimZ, 0, 200, -YAW_LIMIT, YAW_LIMIT);

   
  modeControl();
     
  FlightControl();
   
  analogWrite(blueLed,PWMRANGE-blue);
  analogWrite(redLed, PWMRANGE-red);
  analogWrite(greenLed, green); 

    
    if (sendMpu == true){
 unsigned long currentMillisTrail = millis();
  if(currentMillisTrail - previousMillisTrail >= boundRate) {
    // save the last time you blinked the LED 
    previousMillisTrail = currentMillisTrail;   
 // client.print(strial);
   strial = ("DEL:");
   strial += (0);
   strial += ("#ACC:");
   strial += (round(-rate[0]/100));
   strial += (",");
   strial += (round(rate[1]/100));
   strial += (",");
   strial += (round(-rate[2]/100));
   strial += ("#GYR:");
   strial += (round(-attitude[0]/100));
   strial += (",");
   strial += (round(-attitude[1]/100));
   strial += ("#FIL:");
   strial += (otoMeasure);
   strial += (",");
   strial += (-xMulti.output);
   strial += (",");
   strial += (-yMulti.output);
  client.println(strial);
  strial== "";
  }
    }

    
    }
    }

   // Serial.println("Client disconnected.");
   // client.stop();
 } 



  
}



#endif 
