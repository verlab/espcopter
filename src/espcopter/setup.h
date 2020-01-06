
// Include pf Arduino

#include <Arduino.h>

// Includes of Expressif SDK

extern "C" {
#include <pwm.h>
#include <user_interface.h>
}

////// PWM

// Period of PWM frequency -> default of SDK: 5000 -> * 200ns ^= 1 kHz

#define PWM_PERIOD  256  //256 - 500 - 1024


// PWM channels

#define PWM_CHANNELS 4

// PWM setup (choice all pins that you use PWM)

uint32 io_info[PWM_CHANNELS][3] = {
  // MUX, FUNC, PIN
  //  {PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5,   5}, // D1
  //  {PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4,   4}, // D2
  //  {PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0,   0}, // D3
  //  {PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2,   2}, // D4
  {PERIPHS_IO_MUX_MTMS_U,  FUNC_GPIO14, 14}, // D0
  {PERIPHS_IO_MUX_MTDI_U,  FUNC_GPIO12, 12}, // D1
  {PERIPHS_IO_MUX_MTCK_U,  FUNC_GPIO13, 13}, // D2
  {PERIPHS_IO_MUX_MTDO_U,  FUNC_GPIO15 , 15}, // D3
  // D0 - not have PWM :-(
};

// PWM initial duty: all off

uint32 pwm_duty_init[PWM_CHANNELS];

// Dimmer variables


void scanShields() {
  Wire.pins(4, 5);
  Wire.begin(4, 5);
  Wire.setClock(400000L);
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");
  delay(1500);
  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    delay(50);

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address);
      Serial.println("  !");
      if (address == 22 || address == 41) {
        vl5310xControl = 1;
        Serial.println("  Altidude hold shield was found" + vl5310xControl);
        Serial.println(vl5310xControl);
      }

      if (address == 112 ) {
        multiRangerControl = 1;
        Serial.println("  multi-ranger hold shield was found " + multiRangerControl);
        Serial.println(multiRangerControl);
      }

      if (address == 119 ) {
        bme280Control = 1;
        Serial.println("  BME280 shield was found " + bme280Control);
        Serial.println(bme280Control);
      }

      if (address == 8 ) {
        opticalFlowControl = 1;
        Serial.println("  Optical hold shield was found " + opticalFlowControl);
        Serial.println(opticalFlowControl);
      }

      if (address == 9 ) {
        lpsControl = 1;
        Serial.println("  dwml000 hold shield was found " + lpsControl);
        Serial.println(lpsControl);
      }



      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

}

void calibrationInt() {
  while (calTest == 1 ) {
    unsigned long calMillis = millis();
    if (takeTimeCal = 0) {
      calMillisPre = calMillis;
      takeTimeCal = 1;
    }
    Serial.println(calMillis);
    EEPROM.write(0, 1);
    EEPROM.commit();
    analogWrite(redLed, 0);
    delay(50);
    analogWrite(redLed, PWMRANGE);
    delay(50);
    if (calMillis - calMillisPre > 2500) {
      analogWrite(redLed, PWMRANGE);
      analogWrite(greenLed, 0);
      analogWrite(blueLed, 0); // Blue opened
      EEPROM.write(0, 0);
      EEPROM.commit();
      calTest = 0;
      break;
    }
  }

}



void mainSetup() {


  // Set Pin Mode
  //Serial.begin(921600);// 921600

  //while (! Serial) {
  //  delay(1);
  //}


  analogWriteFreq(20000);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);

  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  digitalWrite(14, LOW);
  digitalWrite(15, LOW);

//  pinMode(D1, OUTPUT);
//  pinMode(D2, OUTPUT);
//  pinMode(D3, OUTPUT);
//  pinMode(D4, OUTPUT);
//  pinMode(D5, OUTPUT);
//  pinMode(D6, OUTPUT);
//  pinMode(D7, OUTPUT);
//  pinMode(D8, OUTPUT);
//
//  digitalWrite(D1, LOW);
//  digitalWrite(D2, LOW);
//  digitalWrite(D3, LOW);
//  digitalWrite(D4, LOW);
//  digitalWrite(D5, LOW);
//  digitalWrite(D6, LOW);
//  digitalWrite(D7, LOW);
//  digitalWrite(D8, LOW);

  ////// Initialize the PWM

  // Initial duty -> all off

  for (uint8_t channel = 0; channel < PWM_CHANNELS; channel++) {
    pwm_duty_init[channel] = 0;
  }

  // Period

  uint32_t period = PWM_PERIOD;

  // Initialize

  pwm_init(period, pwm_duty_init, PWM_CHANNELS, io_info);

  // Commit

  pwm_start();

  digitalWrite(greenLed, 1);
  digitalWrite(blueLed, 0);
  digitalWrite(redLed, 1);

  EEPROM.begin(512);
  scanShields();

  calAHRS = EEPROM.read(0);
  Serial.print("[DRONE] Calibration? ");
  Serial.println(calAHRS);
  ahrs.Initialize(calAHRS);

  if (Trim_DF == 1) {
    EEPROM.write(60, highByte(Trim_Roll_DF));
    EEPROM.write(61, lowByte(Trim_Roll_DF));

    EEPROM.write(62, highByte(Trim_Pitch_DF));
    EEPROM.write(63, lowByte(Trim_Pitch_DF));

    EEPROM.write(64, highByte(Trim_Yaw_DF));
    EEPROM.write(65, lowByte(Trim_Yaw_DF));

    EEPROM.commit();
  }


  Trim_Roll_Bs = word(EEPROM.read(60), EEPROM.read(61));
  Trim_Pitch_Bs = word(EEPROM.read(62), EEPROM.read(63));
  Trim_Yaw_Bs = word(EEPROM.read(64), EEPROM.read(65));

  yaw.SetItermRate(Trim_Yaw_Bs);

  Serial.print("Trim_Roll_Bs: ");
  Serial.print(Trim_Roll_Bs);
  Serial.print(" Trim_Pitch_Bs: ");
  Serial.print(Trim_Pitch_Bs);
  Serial.print(" Trim_Yaw_Bs: ");
  Serial.print(Trim_Yaw_Bs);
  Serial.println(" done Trim_Bs");

  //calibrationInt();

  //setupWiFi();

  roll.SetGain(7.2, 26.0, 0.16, 0.6, 0.9); // roll.SetGain(7.2, 26.0, 0.16,0.6,0.9);
  roll.SetLimit(4500, 200, 3500);

  pitch.SetGain(7.2, 26.0, 0.16, 0.6, 0.9);
  pitch.SetLimit(4500, 200, 3500);

  yaw.SetGain(0.8, 0.3, 0.5, 0.9, 0.0); // (0.8,0.3,0.5,0.9,0.0);
  yaw.SetLimit(4500, 1000, 6000);

  oto.SetGain(0.0, 0.0, 1.8, 1.5, 750.0); // 1.8,3.5,1000.0);
  oto.SetLimit(1000, 1000, 6500);

  yOpt.SetGain(0.0, 0.0, 0.016, 0.05 , 0.0); //0.016(++) ,0.04 ,0.0(~));
  yOpt.SetLimit(4500, 1000, 6000); //3000, 900, 3500

  xOpt.SetGain(0.0, 0.0, 0.016, 0.05 , 0.0); //0.02
  xOpt.SetLimit(4500, 1000, 6000); //3000, 900, 3500

  xMulti.SetGain(0.0, 0.0, 1.6 , 0.4 , 125.0); //  0.9 ,0.4 ,125); // 1.6 ,0.6 ,125.0);
  xMulti.SetLimit(4500, 1000, 6000); //3000, 900, 3500

  yMulti.SetGain(0.0, 0.0, 1.6 , 0.4 , 125.0);
  yMulti.SetLimit(4500, 1000, 6000); //3000, 900, 3500

  xMultiM.SetGain(0.0, 0.0, 1.6 , 0.4 , 125.0); //  0.9 ,0.4 ,125); // 1.6 ,0.6 ,125.0);
  xMultiM.SetLimit(4500, 1000, 6000); //3000, 900, 3500

  yMultiM.SetGain(0.0, 0.0, 1.6 , 0.4 , 125.0);
  yMultiM.SetLimit(4500, 1000, 6000); //3000, 900, 3500


  //xLps.SetGain(0.0, 0.0, 1.2 , 0.4 ,100.0); //  xLps.SetGain(0.0, 0.0, 1.5 , 0.3 ,500.0);  lpsAlpha = 15 loop 30
  //xLps.SetLimit(4500, 1000, 6000); //

  xLps.SetGain(0.0, 0.0, 0.12 , 0.0 , 3.0);
  xLps.SetLimit(4500, 1000, 6000);

  yLps.SetGain(0.0, 0.0, 1.5 , 0.0 , 3.0);
  yLps.SetLimit(4500, 1000, 6000); //

  xGOT.SetGain(0.0, 0.0, 1.3, 0.3 , 125.0);
  xGOT.SetLimit(4500, 1000, 6000);

  yGOT.SetGain(0.0, 0.0, 1.3, 0.3 , 125.0);
  yGOT.SetLimit(4500, 1000, 6000); //

}
