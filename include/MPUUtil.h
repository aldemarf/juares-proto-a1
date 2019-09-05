#ifndef __MPUUTIL_H__
#define __MPUUTIL_H__

#include "SDUtil.h"
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

const uint8_t NUM_SAMPLES = 100;

class MPUUtil {
    public:
        static MPUUtil* getInstance();
        void writeToFile();
        void readFromSensor();
        void setup();
        void wakeup();
    private:
        MPUUtil();
        MPUUtil(const MPUUtil&) = delete;
        MPUUtil& operator=(const MPUUtil&) = delete;
        static MPUUtil* pInstance;
        MPU6050 mpu;
        SDUtil* sd;
        // MPU-6050 constants
        static const uint8_t INTERRUPT_PIN = 2;
        // TODO: considering migrate this to a class, research the best solution
        struct mpu_samples_t {
            time_t ts;
            Quaternion q;
            int16_t gX;
            int16_t gY;
            int16_t gZ;
            int16_t aX;
            int16_t aY;
            int16_t aZ;
        } mpu_samples[NUM_SAMPLES];
        // RTC_DATA_ATTR uint8_t cur_sample = 0;
};


#endif