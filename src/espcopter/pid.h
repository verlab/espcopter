#include <Arduino.h>

class PID {
  public:

    void SetGain(float kp, float ki, float kpo, float kio, float kdo);
    void SetLimit(int16_t error_limit, int16_t i_limit, int16_t output_limit);
    void SetInit(int32_t p, int32_t d, int32_t i);
    void Setp(int32_t output_);
    void SetKI(float ki);
    void SetKDO(float kd);
    void SetControl(int cn);
    void SetItermRate(int32_t output_);
    int16_t GetItermRate();
    int16_t GetItermRateBase();
    int GetKPO();
    void compute(int whichIs, int32_t TARGET_ANGLE, int32_t THROTTLE, int32_t TRIM , int32_t MEASURED_ANGLE, int32_t MEASURED_RATE);
    int16_t output = 0; //throttle
    int32_t itermRate = 0;
    float KP = 0, KI = 0, KD = 0, KPO = 0, KIO = 0, KDO = 0 , KIO_ = 0, control = 0;
  private:
    int32_t lastErr = 0;
    int16_t timenow3 = 0, timeprev3 = 0;
    int16_t timenow2 = 0, timeprev2 = 0;
    int16_t timenow = 0, timeprev = 0;
    int16_t ERROR_LIMIT = 0, I_LIMIT = 0, OUTPUT_LIMIT = 0;
    int32_t PTERM = 0, ITERM = 0 , _output = 0 ;
    int32_t   P = 0,  D = 0, delta = 0;
    int32_t I = 0, _delta = 0, _MEASURED_RATE = 0;
    //int whichIs;TARGET_RATE
    int32_t delta2 = 0, _ANGLE_ERROR = 0;
    int32_t  YAW_RATE = 0;
    int32_t itermRateBase = 0;
    int moveCommandX = 1;
    int moveCommandY = 1;
    int firstTime = 1;
};
