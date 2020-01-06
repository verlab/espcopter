#include "AHRS.h"
#include "Wire.h"

int previousDegree;
float angle = 0;
float heading = 0;
void AHRS::Initialize(int calAHRS)
{
  // Start I2C Communication
  //Wire.begin();
  Wire.pins(4, 5);
  Wire.begin(4, 5);
  Wire.setClock(400000L);
  //Wire.setClockStretchLimit(500L);

  // Set accelerometers low pass filter at 5Hz
  writeBIT(29, 0x06, MPU9250_ADDRESS);
  // Set gyroscope low pass filter at 5Hz
  writeBIT(26, 0x06, MPU9250_ADDRESS);
  // Configure gyroscope range
  writeBIT(27, GYRO_FULL_SCALE_2000_DPS, MPU9250_ADDRESS);
  // Configure accelerometers range
  writeBIT(28, ACC_FULL_SCALE_2_G, MPU9250_ADDRESS);
  // Set by pass mode for the magnetometers
  writeBIT(0x37, 0x02, MPU9250_ADDRESS);
  // Request continuous magnetometer measurements in 16 bits
  //writeBIT(0x0A, 0x16,MAG_ADDRESS);
  delay(20);

  writeBIT(0x0A, 0x00 , MAG_ADDRESS); // Power down magnetometer
  delay(20);
  writeBIT(0x0A  , 0x0F, MAG_ADDRESS); // Enter Fuse ROM access mode
  delay(20);
  uint8_t rawData[3];
  I2Cread(MAG_ADDRESS, 0x10, 3, rawData);
  destination[0] =  (float)(rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128) / 256. + 1.;
  destination[2] =  (float)(rawData[2] - 128) / 256. + 1.; //28 , 238 , 254 , 0.61 , 1.43 , 1.49

  Serial.print("Sensitivity: ");
  Serial.print(destination[0]);
  Serial.print(" , ");
  Serial.print(destination[1]);
  Serial.print(" , ");
  Serial.println(destination[2]);
  delay(20);

  writeBIT(0x0A, 0x00, MAG_ADDRESS); // Power down magnetometer
  delay(20);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeBIT(0x0A, Mscale << 4 | Mmode, MAG_ADDRESS); // Set magnetometer data resolution and sample ODR
  delay(20);


  if (calAHRS == 1) {
    calibration();
  } //else {
    loadParameter();
  //}


  Serial.print(" destination20: ");
  Serial.println(destination2[0]);
  Serial.print(" destination21: ");
  Serial.println(destination2[1]);
  Serial.print(" MagX: ");
  Serial.println(magOffsetMotorEpr[0]);
  Serial.print(" MagY: ");
  Serial.println(magOffsetMotorEpr[1]);
  Serial.print(" Offx: ");
  Serial.println(magOffset[0]);
  Serial.print(" Offy: ");
  Serial.println(magOffset[1]);
  Serial.print(" Offz:");
  Serial.println(magOffset[2]);
  Serial.print(" GYROX: ");
  Serial.println(MEAN_GYRO[0]);
  Serial.print(" GYROY: ");
  Serial.println(MEAN_GYRO[1]);
  Serial.print(" GYROZ: ");
  Serial.println(MEAN_GYRO[2]);
}

String AHRS::showParameter(){
  char buf2 [200];
  sprintf(buf2, "\33[93m[Calibration] Destination: [%f, %f], MagOffsetMotor: [%f, %f], MagOffset: [%f, %f, %f], MeanGyro: [%f, %f, %f]  \33[0m", destination2[0], destination2[1], magOffsetMotorEpr[0], magOffsetMotorEpr[1], magOffset[0], magOffset[1], magOffset[2], MEAN_GYRO[0], MEAN_GYRO[1], MEAN_GYRO[2]);
  return String(buf2);
}

void AHRS::loadParameter(){
  Serial.print("Read Eeprom...");
    if (EEPROM.read(4) == 1) {
      MEAN_GYRO[0] = (int32_t)-EEPROM.read(1);
    } else {
      MEAN_GYRO[0] = (int32_t)EEPROM.read(1);
    }
    if (EEPROM.read(5) == 1) {
      MEAN_GYRO[1] = (int32_t)-EEPROM.read(2);
    } else {
      MEAN_GYRO[1] = (int32_t)EEPROM.read(2);
    }
    if (EEPROM.read(6) == 1) {
      MEAN_GYRO[2] = (int32_t)-EEPROM.read(3);
    } else {
      MEAN_GYRO[2] = (int32_t)EEPROM.read(3);
    }


    magOffset[0] = (int16_t) word(EEPROM.read(10), EEPROM.read(11));
    magOffset[1] = (int16_t) word(EEPROM.read(20), EEPROM.read(21));
    magOffset[2] = (int16_t) word(EEPROM.read(30), EEPROM.read(31));

    magOffsetMotorEpr[0] = (int16_t) word(EEPROM.read(40), EEPROM.read(41));
    magOffsetMotorEpr[1] = (int16_t) word(EEPROM.read(50), EEPROM.read(51));

    magOffsetMotorEpr[2] = (int16_t) word(EEPROM.read(45), EEPROM.read(46));
    magOffsetMotorEpr[3] = (int16_t) word(EEPROM.read(55), EEPROM.read(56));

    destination2[0] = (float) word(EEPROM.read(12), EEPROM.read(13)) / 100;
    destination2[1] = (float) word(EEPROM.read(14), EEPROM.read(15)) / 100;
  }

void AHRS::calibration(){
    //EEPROM.write(0, 0);
    //EEPROM.commit();
    delay(2000);
    Serial.println("Calibration Progress Started");
//    analogWriteFreq(20000);
    analogWrite(redLed, 0);
    analogWrite(greenLed, PWMRANGE);
    analogWrite(blueLed, 0);

    magCalStage = 1;
    magCorrection(1000);
    magCalStage = 0;

    Serial.println("Drone is calibrated!");
    analogWrite(redLed, PWMRANGE);
    analogWrite(greenLed, PWMRANGE);
    analogWrite(blueLed, PWMRANGE);

    EEPROM.write(35, highByte(0));
    EEPROM.write(36, lowByte(0));

    EEPROM.write(37, highByte(0));
    EEPROM.write(38, lowByte(0));

    EEPROM.write(42, highByte(0));
    EEPROM.write(43, lowByte(0));

    EEPROM.commit();


    calAHRS = 0;
}

void AHRS::writeBIT(int addr, int data, int adrrTrans) {
  Wire.beginTransmission(adrrTrans);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}
void AHRS::I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) {
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}

float AHRS::rad2deg(float rad) {
  float output = rad * (180.0 / PI);
  return output;
}

float AHRS::deg2rad(float deg) {
  float output = deg * (PI / 180.0);
  return output;
}

void AHRS::compute(float attitude[3], float rate[3], float attitudeRadian[3], float rateRadian[3] ) {
  timenow = millis();
  dt = (float)(timenow - timeprev) / 978.;
  timeprev = timenow;

  if (dt > -1) {
    // Get Raw Data From IMU

    I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
    accel[0] = (Buf[0] << 8 | Buf[1]);
    accel[1] = (Buf[2] << 8 | Buf[3]);
    accel[2] = Buf[4] << 8 | Buf[5];

    // Gyroscope
    gyro_int[0] = (Buf[8] << 8 | Buf[9]) - MEAN_GYRO[0];
    gyro_int[1] = (Buf[10] << 8 | Buf[11]) - MEAN_GYRO[1];
    gyro_int[2] = Buf[12] << 8 | Buf[13] - MEAN_GYRO[2];

    for (i = 0; i < 3 ; i++) {
      // Get Gyro
      gyro[i] = (gyro_int[i] / GYRO_LSB) * DEG2RAD;
    }

    // Normalize Accelerometer
    normalize(accel_angle, accel);

    // Calculate deltaAngle
    gyro_angle_x = gyro_angle_x + (gyro_angle_y * gyro[2] - gyro_angle_z * gyro[1]) * dt;
    gyro_angle_y = gyro_angle_y + (gyro_angle_z * gyro[0] - gyro_angle_x * gyro[2]) * dt;
    gyro_angle_z = gyro_angle_z + (gyro_angle_z * gyro[1] - gyro_angle_y * gyro[0]) * dt;

    // Apply Complementary Filter
    // Apply when |a| - 1 < 0.10 to eliminate other forces
    if ((abs(norm) / 16384.) - 1 <= 0.15) { //0.15
      gyro_angle_x = A_A * gyro_angle_x + (1 - A_A) *  accel_angle[0];
      gyro_angle_y = A_A * gyro_angle_y + (1 - A_A) *  accel_angle[1];
      gyro_angle_z = A_A * gyro_angle_z + (1 - A_A) *  accel_angle[2];
    }

    // Convert
    attitude[0] = atan2(gyro_angle_y, sqrt(gyro_angle_x * gyro_angle_x + gyro_angle_z * gyro_angle_z)); // ROLL
    attitude[1] = atan2(gyro_angle_x, gyro_angle_z); // PITCH

    // Calculate Euler Rates
    rate[0] =  (gyro[0] + ( gyro[1] * attitude[0] * attitude[1]) + ( gyro[2] * (1 - (attitude[0] * attitude[0]) / 2) * attitude[1]));
    rate[1] =  (gyro[1] * (1 - ((attitude[0] * attitude[0]) / 2)) -  gyro[2] * attitude[0]);
    rate[2] = gyro[2];

    rateRadian[0] =  rate[0];
    rateRadian[1] =  rate[1];
    rateRadian[2] =  rate[2];

    rate[0] =  rate[0] * 100 * RAD2DEG;
    rate[1] =  rate[1] * 100 * RAD2DEG;
    rate[2] =  rate[2] * 100 * RAD2DEG;


    float alpha = 0.85;
    for (i = 0; i < 3 ; i++) {
      rate[i] = (alpha * (float)rate[i]) + ((1 - alpha) * (float)_rate[i]);
      _rate[i] = rate[i];
    }

    attitudeRadian[0] =  attitude[0];
    attitudeRadian[1] =  attitude[1];

    attitude[0] *= 100 * RAD2DEG;
    attitude[1] *= 100 * RAD2DEG;

    float alpha2 = 0.95;
    for (i = 0; i < 2 ; i++) {
      attitude[i] = (alpha2 * (float)attitude[i]) + ((1 - alpha2) * (float)_attitude[i]);
      _attitude[i] = attitude[i];
    }
  } else {
    Serial.print((float)dt);
    Serial.println("Error AHRS");
  }
}

void AHRS::normalize(float output[3], int16_t input[3])
{
  norm = sqrt(input[0] * input[0] + input[1] * input[1] + input[2] * input[2]);
  output[0] = input[0] / (float)norm;
  output[1] = input[1] / (float)norm;
  output[2] = input[2] / (float)norm;
}

void AHRS::setZero()
{
  gyro_angle_x = 0;
  gyro_angle_y = 0;
  gyro_angle_z = 0;
}

int16_t  AHRS::readTemp() {
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  I2Cread(MPU9250_ADDRESS, 0x41, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}



void AHRS::gyroCorrection(uint16_t itinary) {
  for (i = 0; i < itinary; i++) {
    I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
    gyro_int[0] = (Buf[8] << 8 | Buf[9]);
    gyro_int[1] = (Buf[10] << 8 | Buf[11]);
    gyro_int[2] = Buf[12] << 8 | Buf[13];
    SUM_GYRO[0] += gyro_int[0];
    SUM_GYRO[1] += gyro_int[1];
    SUM_GYRO[2] += gyro_int[2];
  }
  Serial.print("MEAN GYRO: ");
  for (i = 0; i < 3; i++) {
    MEAN_GYRO[i] = SUM_GYRO[i] / itinary;
    if (MEAN_GYRO[i] < 0) {
      EEPROM.write(4 + i, 1);
      EEPROM.write(1 + i, -(SUM_GYRO[i] / itinary));
      Serial.print(-(SUM_GYRO[i] / itinary));
    } else {
      EEPROM.write(4 + i, 0);
      EEPROM.write(1 + i, SUM_GYRO[i] / itinary);
      Serial.print(SUM_GYRO[i] / itinary);
    }
    Serial.print(", ");
  }
  Serial.println();
  
}

void AHRS::magCorrection(uint16_t itinary) {
  magOffset[0] = 0;
  magOffset[1] = 0;
  magOffset[2] = 0;
  // destination[0] = 1;
  //destination[1] = 1;
  // destination[2] = 1;
  magOffsetMotorFnl[0] = 0;
  magOffsetMotorFnl[1] = 0;

  float attitudeFake[3] = {0};

  float  rateFake[3] = {0};
  float magOffsetMotorFnlFake[3] = {0};
  analogWrite(redLed, 0);
  analogWrite(greenLed, PWMRANGE);
  analogWrite(blueLed, 100);
  delay(1000);

  headingMag(rateFake, attitudeFake, degree , 0.0);

  minX = degree[1];
  maxX = degree[1];
  minY = degree[2];
  maxY = degree[2];

  Serial.println("Calibration Mag... Turn the drone cw!");
  for (i = 0; i < itinary; i++) {
    I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
    headingMag(rateFake, attitudeFake, degree , 0);
    Serial.print("Degree: ");
    Serial.print(degree[1]);
    Serial.print(", ");
    Serial.println(degree[2]);
    if (degree[1] < minX) minX = degree[1];
    if (degree[1] > maxX) maxX = degree[1];

    if (degree[2] < minY) minY = degree[2];
    if (degree[2] > maxY) maxY = degree[2];


    Serial.print("minX = ");
    Serial.print(minX);
    Serial.print("  ,  ");
    Serial.print(maxX);
    Serial.print("  ,  ");
    Serial.print(minY);
    Serial.print("  ,  ");
    Serial.print(maxY);
    Serial.print("  ,  ");
    Serial.print(degree[1]);
    Serial.print("  ,  ");
    Serial.print(degree[2]);
    Serial.print("  ,  ");
    Serial.println(degree[3]);
    delay(12);
  }
  Serial.println("Mag is done!");

  analogWrite(redLed, 0);
  analogWrite(greenLed, PWMRANGE);
  analogWrite(blueLed, 400);

  delay(2000);
  Serial.println("Gyro Correction... Do not touch the drone! ");
  gyroCorrection(2048);
  Serial.println("Gyro is done!");

  analogWrite(redLed, 0);
  analogWrite(greenLed, PWMRANGE);
  analogWrite(blueLed, 600);
  

  // magOffset[0] = (((maxX - minX)/2)- maxX);
  // magOffset[1] = -(maxY + ((minY - maxY)/2));

  magOffset[0] = (maxX + minX) / 2;
  magOffset[1] = (maxY + minY) / 2;

  mag_scale[0]  = (maxX - minX) / 2; // get average x axis max chord length in counts
  mag_scale[1]  = (maxY - minY) / 2; // get average y axis max chord length in counts

  float avg_rad  = (mag_scale[0] + mag_scale[1]) / 2;


  destination2[0] = avg_rad / ((float)mag_scale[0]) ;
  destination2[1] = avg_rad / ((float)mag_scale[1]) ;

  Serial.print(" 1 offX: ");
  Serial.print(magOffset[0]);
  Serial.print(" 1 offY: ");
  Serial.print(magOffset[1]);
  Serial.print(" 1 offZ: ");
  Serial.println(magOffset[2]);


  while (1) {
    headingMag(rateFake, attitudeFake, degree , 0 );
    Serial.print("magOffsetMotorCount: ");
    Serial.println(magOffsetMotorCount);
    Serial.print("degreeX = ");
    Serial.print(degree[1]);
    Serial.print(", ");
    Serial.println(degree_[1]);
    Serial.print(" degreeY = ");
    Serial.print(degree[2]);
    Serial.print(", ");
    Serial.print(degree[2]);
    Serial.print(", ");
    Serial.println(magOffsetMotorCount);
    if ( magOffsetMotorCount < 200 && degree_[2] != degree[2] && degree_[1] != degree[1] ) {
      magOffsetMotorCount += 1;
      magOffsetMotor[0] += degree[1];
      magOffsetMotor[1] += degree[2];
      degree_[1] = degree[1];
      degree_[2] = degree[2];
      Serial.print("degreeX = ");
      Serial.print(degree[1]);
      Serial.print(" degreeY = ");
      Serial.print(degree[2]);
      Serial.println();
    } else if ( magOffsetMotorCount >= 200) {
      sumMagOffsetMotor[0] = magOffsetMotor[0] / 200;
      sumMagOffsetMotor[1] = magOffsetMotor[1] / 200;
      break;
    }
  }
  Serial.println();
  Serial.println("ORT = ");
  Serial.println();
  Serial.print("sumMagX = ");
  Serial.print(sumMagOffsetMotor[0]);
  Serial.print(", sumMagY = ");
  Serial.println(sumMagOffsetMotor[1]);

  /*analogWrite(redLed, 0);
  analogWrite(greenLed, PWMRANGE);
  analogWrite(blueLed, 900);*/
  
  analogWriteFreq(20000);
  for (int i = 0; i < 30; i++) {
    analogWrite(14, i * 10);
    analogWrite(15, i * 10);
    analogWrite(12, i * 10);
    analogWrite(13, i * 10);
    delay(25);
  }
  while (1) {
    Serial.println("Starting Killer loop");
    analogWrite(14, 300);
    analogWrite(15, 300);
    analogWrite(12, 300);
    analogWrite(13, 300);
    headingMag(rateFake, attitudeFake, degree , 0.0);
    Serial.print("degree: ");
    Serial.print(degree[0]);
    Serial.print(", ");
    Serial.print(degree_[0]);
    Serial.print(degree[1]);
    Serial.print(", ");
    Serial.println(degree_[1]);
    if ( magOffsetMotorCount2 < 5 && degree_[0] != degree[0] && degree_[1] != degree[1] ) {
      magOffsetMotorCount2 += 1;
      degree_[1] = degree[1];
      degree_[2] = degree[2];
      Serial.print("degreeX = ");
      Serial.print(degree[1]);
      Serial.print(" degreeY = ");
      Serial.print(degree[2]);
      Serial.println();
    } else if ( magOffsetMotorCount2  >= 5) {
      sumMagOffsetMotor2[0] = degree[1];
      sumMagOffsetMotor2[1] = degree[2];
      analogWrite(14, 0);
      analogWrite(15, 0);
      analogWrite(12, 0);
      analogWrite(13, 0);
      break;
    }
  }
  Serial.println();
  Serial.println("ORT 2 = ");
  Serial.println();
  Serial.print("sumMagX = ");
  Serial.print(sumMagOffsetMotor2[0]);
  Serial.print(" sumMagY = ");
  Serial.print(sumMagOffsetMotor2[1]);
  Serial.println();

  magOffsetMotorEpr[0] = (sumMagOffsetMotor[0] - sumMagOffsetMotor2[0]);
  magOffsetMotorEpr[1] = (sumMagOffsetMotor[1] - sumMagOffsetMotor2[1]);

  Serial.println();
  Serial.println("LAST 2 = ");
  Serial.println();
  Serial.print(" sumMaglastY= ");
  Serial.print(magOffsetMotorEpr[0]);
  Serial.print(" sumMaglastY = ");
  Serial.print(magOffsetMotorEpr[1]);
  Serial.println();

  analogWrite(redLed, 0);
  analogWrite(greenLed, PWMRANGE);
  analogWrite(blueLed, 0);

  EEPROM.write(12, highByte(int(destination2[0] * 100)));
  EEPROM.write(13, lowByte(int(destination2[0] * 100)));

  EEPROM.write(14, highByte(int(destination2[1] * 100)));
  EEPROM.write(15, lowByte(int(destination2[1] * 100)));

  EEPROM.write(10, highByte(magOffset[0]));
  EEPROM.write(11, lowByte(magOffset[0]));

  EEPROM.write(20, highByte(magOffset[1]));
  EEPROM.write(21, lowByte(magOffset[1]));

  EEPROM.write(30, highByte(magOffset[2]));
  EEPROM.write(31, lowByte(magOffset[2]));

  EEPROM.write(40, highByte(magOffsetMotorEpr[0]));
  EEPROM.write(41, lowByte(magOffsetMotorEpr[0]));

  EEPROM.write(50, highByte(magOffsetMotorEpr[1]));
  EEPROM.write(51, lowByte(magOffsetMotorEpr[1]));


}

void AHRS::headingMag(float attitude_rate[3], float  input[3], float degree[4], float throttle) {

  I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);

  if (ST1 & 0x01) {
    //Serial.println("---- Mag i2c Works!");
    //analogWrite(greenLed, 1000);    
    uint8_t Mag[7];
    I2Cread(MAG_ADDRESS, 0x03, 7, Mag);
    // Create 16 bits values from 8 bits data
    // Magnetometer
    mag_int[0] = (Mag[3] << 8 | Mag[2]);
    mag_int[1] = (Mag[1] << 8 | Mag[0]);
    mag_int[2] = (Mag[5] << 8 | Mag[4]);

    /*
      if(throttle == 0){
      magOffset3_ = magOffset[2];
      magCaCoefficient=0;
      }else{
      magCaCoefficient  =  map(100*constrain(magOffset3_  - magOffset[2],0,1500),0,150000,0.00,1000.00);
      magCaCoefficient = magCaCoefficient/100;
      }
    */




    if (input[0] < 0.1 && input[0] > -0.1 && input[1] > -0.1 && input[1] < 0.1 ) { //&&  throttle == 0
      if ((mag_int[2]* mRes * destination[2]) < 600) {
        magOffset[2] = 600 - (mag_int[2] * mRes * destination[2]);
      } else {
        magOffset[2] = -((mag_int[2] * mRes * destination[2]) - 600 );
      }
    }




    magOffsetMotorFnl2[0] =  magOffsetMotorEpr[0] * (((float)map(100 * constrain(throttle, 0, 700), 0, 70000, 0, 100)) / 100);
    magOffsetMotorFnl2[1] =  magOffsetMotorEpr[1] * (((float)map(100 * constrain(throttle, 0, 700), 0, 70000, 0, 100)) / 100);



    mag[0] = (mag_int[0] * mRes * destination[0]) - magOffset[0] + magOffsetMotorFnl2[0];
    mag[1] = (mag_int[1] * mRes * destination[1]) - magOffset[1] + magOffsetMotorFnl2[1];
    mag[2] = (mag_int[2] * mRes * destination[2]) + magOffset[2];

    // mag[0] *= destination2[0];
    // mag[1] *= destination2[1];

    degree[1] =  mag[0];
    degree[2] =  mag[1];
    degree[3] =  mag[2];

    float roll;
    float pitch;

    pitch = asin(input[1]);
    roll = -asin(input[0]);

    float cosRoll = cos(roll);
    float sinRoll = sin(roll);
    float cosPitch = cos(pitch);
    float sinPitch = sin(pitch);

    float Xh =  mag[0] * cosPitch +  mag[2] * sinPitch;
    float Yh =  mag[0] * sinRoll * sinPitch +  mag[1] * cosRoll -  mag[2] * sinRoll * cosPitch;

    float heading1 = atan2(Yh, Xh);

    float declinationAngle2 = (4.0 + (26.0 / 60.0)) / (180 / M_PI);

    heading1 += declinationAngle2;
    // Correct for heading < 0deg and heading > 360deg
    if (heading1 < 0) {
      heading1 += 2 * PI;
    }
    if (heading1 > 2 * PI) {
      heading1 -= 2 * PI;
    }

    previousHeadingDegrees = headingDegrees;

    headingDegrees = heading1 * 180 / M_PI;


    unsigned long headingCurrentMillis = millis();
    /*
      if(throttle < 100){
      headingPreviousMillis = headingCurrentMillis;
      }

      if(headingCurrentMillis - headingPreviousMillis < 1000){
      headingDegreesDefoult = headingDegrees;
      }
    */

    if (throttle < 100) {
      headingDegreesDefoult = headingDegrees;
      HeadingLoop = 0;
    }

    int turnRight;
    int turnLeft ;

    if ( HeadingCount == true) {

      if (headingDegreesDefoult - previousHeadingDegrees < 30 &&  headingDegreesDefoult - previousHeadingDegrees > -30) {
        errorDirection = headingDegreesDefoult - previousHeadingDegrees;
      }


      if ( errorDirection > 0  ) {

        if (previousHeadingDegrees - headingDegrees  < -300 || turnLeft == 1) { // 360 to 0
          HeadingLoop = -360;
          turnLeft = 1;
        }

        if (previousHeadingDegrees - headingDegrees  > 300 && turnLeft == 1) { // 0 to 360
          turnLeft = 0;
          HeadingLoop = 0;
        }
      } else {

        if ( ( previousHeadingDegrees - headingDegrees  > 300 || turnRight == 1 ) ) { // 360 to 0
          HeadingLoop = 360;
          turnRight = 1;
        }

        if (previousHeadingDegrees - headingDegrees  < -300 && turnRight == 1) { // 0 to 360
          turnRight = 0;
          HeadingLoop = 0;
        }

      }
    }
    HeadingCount = true;


    degree[0] = headingDegrees  - headingDegreesDefoult + HeadingLoop ; //MEAN_GYRO

/*
    
      Serial.print("Heading Degrees  ");
      Serial.println(headingDegrees);

    
        float degree_[2];
        degree[0] = (0.85*(float)degree[0]) + ((1-0.85)*(float)degree_[0]);
        degree_[0] = degree[0];

    
        Serial.print("degree[0]: ");
       Serial.println(degree[0]);
       Serial.print("headingDegrees: ");
       Serial.println(headingDegrees);
       Serial.print("errorDirection: ");
       Serial.println( errorDirection );
       Serial.print("dHeading Degrees: ");
       Serial.println(previousHeadingDegrees - headingDegrees );
       Serial.print("HeadingLoop: ");
       Serial.println(HeadingLoop);  


    Serial.print("Mag: ");
      Serial.print(mag[0]);
      Serial.print(" , ");
      Serial.print(mag[1]);
      Serial.print(" , ");
      Serial.print(mag[2]);
      Serial.print("  ,  ");
      Serial.print(destination2[0]);
      Serial.print("  ,  ");
      Serial.print(magOffset[0]);
      Serial.print("  ,  ");
      Serial.print(headingDegrees); // -0.04 , 0.00 , -66  ,  389  ,  924  ,  104.06  ,  109.10
      Serial.println(); 
    
      Serial.print("MEAN:");
      Serial.println(MEAN_GYRO[0]);
      Serial.print("MEAN:");
      Serial.println(MEAN_GYRO[1]);
      Serial.print("MEAN:");
      Serial.println(MEAN_GYRO[2]); //offz:-285 offX:-1829 offY:479



      Serial.print("magOffsetMotorEpr: ");
      Serial.print(magOffsetMotorEpr[0]);
      Serial.print(" , ");
      Serial.print(magOffsetMotorEpr[1]);
      Serial.print("  ,  ");
      Serial.print(magOffsetMotorFnl2[0]);
      Serial.print(" , ");
      Serial.println(magOffsetMotorFnl2[1]);
    

       Serial.print("throttle: ");
       Serial.println(throttle);
       Serial.print("mag[0]: ");
       Serial.println(mag[0]);
       Serial.print("mag[1]: ");
       Serial.println(mag[1]);
       Serial.print("mag[2]: ");
       Serial.println(mag[2] );
       Serial.print("headingDegrees: ");
       Serial.println(headingDegrees); // -0.04 , 0.00 , -66  ,  389  ,  924  ,  104.06  ,  109.10
*/
    
  } else{
    Serial.println("---- Mag i2c failed!");
    //analogWrite(greenLed, 2000);
    /*delay(500);
    analogWrite(greenLed, 2000);
    delay(500);*/
    }
}
