//******************* AHRS Parameter ********************//

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C
#define VL53L0X_ADDRESS  0x29 
#define VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N                   0x00BF

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define GYRO_RANGE 0x18                // GYRO Range : -2000DPS  ~ 2000DPS 
#define ACCEL_RANGE 0x00                // ACCEL Range : -2g ~ 2g
#define MPU6050_DLPF 0x02               // DLP : 98mHz
#define COMPLEMENTARY_SAMPLE 0.001      // Sample Rate : 1000Hz = 1Khz
#define ACCEL_LSB 16384.                // Accelerometer LSB
#define GYRO_LSB 16.4                 // Gyro LSB
#define DEG2RAD PI/180.
#define RAD2DEG 180./PI
#define A_A 0.9995 //99.95 //99.95
#define TT 978. //95 
     
#define mRes 1.5

#define MAG_XOUT_H 0x03
#define HMC 0x1E

// MPU-6050 Parameter
#define MPU6050 0x68
#define XA_OFFS_USRH 0x06
#define XA_OFFS_USRL 0x07
#define YA_OFFS_USRH 0x08
#define YA_OFFS_USRL 0x09
#define ZA_OFFS_USRH 0x0A
#define ZA_OFFS_USRL 0x0B
#define SELF_TEST_X 0x0D
#define SELF_TEST_Y 0x0E
#define SELF_TEST_Z 0x0F
#define SELF_TEST_A 0x10
#define SMPLRT_DIV 0x19
#define XG_OFFS_USRH 0x13
#define XG_OFFS_USRL 0x14
#define YG_OFFS_USRH 0x15
#define YG_OFFS_USRL 0x16
#define ZG_OFFS_USRH 0x17
#define ZG_OFFS_USRL 0x18
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define FIFO_EN 0x23
#define INT_ENABLE 0x38
#define INT_STATUS 0x3A
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define FIFO_COUNT_H 0x72
#define FIFO_COUNT_L 0x73
#define FIFO_R_W 0x74
#define WHO_AM_I 0x75
//4.2....0.1......1.1
//******************** PID Parameter ********************//



//******************** CONTROL Parameter ********************//
#define ROLL_LIMIT 1250 //1750
#define PITCH_LIMIT 1250 //1750
#define YAW_LIMIT 6000 // 1750
