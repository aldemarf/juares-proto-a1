#ifndef __MPUUTIL_H__
#define __MPUUTIL_H__

#include "SDUtil.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

const uint8_t NUM_SAMPLES = 100;

class MPUUtil {
    public:
        static MPUUtil* getInstance();
        void write_data();
        void read();
        void setup();
        void wake();
    private:
        MPUUtil();
        MPUUtil(const MPUUtil&) = delete;
        MPUUtil& operator=(const MPUUtil&) = delete;
        static MPUUtil* pInstance;
        SDUtil* sd;
        struct mpu_samples_t {
            uint32_t ts;
            Quaternion q;
            int16_t gX;
            int16_t gY;
            int16_t gZ;
            int16_t aX;
            int16_t aY;
            int16_t aZ;
        } mpu_samples[NUM_SAMPLES];
        RTC_DATA_ATTR uint8_t cur_sample = 0;
};


#endif