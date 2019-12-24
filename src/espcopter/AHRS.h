#include <Arduino.h>
#include "Parameter.h"
#include <Wire.h>
#include <EEPROM.h>

class AHRS {
  public:

    void Initialize(int calAHRS);
    float rad2deg(float rad);
    float deg2rad(float deg);
    int16_t readTemp();
    void writeBIT(int addr, int data, int adrrTrans);
    void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);
    void compute(float attitude[3], float rate[3] , float attitudeRadian[3] , float rateRadian[3]);
    void normalize(float output[3], int16_t input[3]);
    void calibration();
    void loadParameter();
    void setZero();
    char* showParameter();
    void headingMag(float attitude_rate[3], float  input[3], float degree[4], float throttle );
  private:
    void gyroCorrection(uint16_t itinary);
    void magCorrection(uint16_t itinary);
    int32_t SUM_GYRO[3] = {0}, MEAN_GYRO[3] = {0};
    int16_t Filter_P[3] = {0}, Filter_I[3] = {0}, Filter_SUM[3] = {0}, delta = 0, _angle[3] = {0};
    float _rate[3] = {0}, test = 0, rate[3] = {0};
    float _w = 0, ic = 0.001, I = 0;
    float dt = 0, timeprev = 0, timenow = 0;
    int i, j = 0;
    int16_t accel[3] = {0}, gyro_int[3] = {0} , mag[3] = {0}, mag_int[3] = {0};
    float accel_angle[3] = {0}, angle_acc[3] = {0};
    float gyro[3] = {0}, degree[3] = {0};
    float gyro_angle_x, gyro_angle_y, gyro_angle_z; // Estimated Gravity Vector
    int16_t norm = 0;
    double attitude[3] = {0}, _attitude[3]  = {0};
    uint8_t ST1;
    uint8_t Buf[14];

    float destination[3] = {1, 1, 1};
    float destination2[3] = {1, 1, 1};

    float avg_delta, scale_x, scale_y;

    unsigned long headingPreviousMillis = 0;
    boolean takeTimeHeading = true;
    enum Ascale {
      AFS_2G = 0,
      AFS_4G,
      AFS_8G,
      AFS_16G
    };

    enum Gscale {
      GFS_250DPS = 0,
      GFS_500DPS,
      GFS_1000DPS,
      GFS_2000DPS
    };

    enum Mscale {
      MFS_14BITS = 0, // 0.6 mG per LSB
      MFS_16BITS      // 0.15 mG per LSB
    };


    uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
    uint8_t Gscale = GFS_250DPS;
    uint8_t Ascale = AFS_2G;



    uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
    int16_t magOffset[3] = {0};
    int16_t mag_scale[3] = {0};
    int16_t magOffset3_ , magOffset3;
    int16_t magxOffsetMotor = 0, magyOffsetMotor  = 0, magzOffsetMotor  = 0;
    float magCaCoefficient;
    float throttle;
    float magDegree[4] = {0};
    float minX, maxX, minY, maxY;
    int calAHRS;
    float headingDegreesDefoult = 0;

    int firstStage = 0;
    int magCalStage = 0;
    float magOffsetMotorCount = 0;
    float magOffsetMotor[3] = {0};
    float sumMagOffsetMotor[3] = {0};

    float degree_[4] = {1, 1, 1, 1};

    float magOffsetMotorCount2 = 0;
    float magOffsetMotor2[3] = {0};
    float sumMagOffsetMotor2[3] = {0};

    int16_t magOffsetMotorFnl[3] = {0};
    int16_t magOffsetMotorFnl2[3] = {0};

    int16_t magOffsetMotorEpr[2] = {0};



    float previousHeadingDegrees = 0;
    float  headingDegrees = 0;
    int HeadingLoop = 0;
    boolean HeadingCount = false;

    int errorDirection = 0;

#define blueLed 16
#define redLed 2
#define greenLed 0

};
