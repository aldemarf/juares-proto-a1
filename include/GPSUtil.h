#ifndef __GPSUTIL_H__
#define __GPSUTIL_H__

#include <TinyGPS++.h>

class GPSUtil {
    public:
        static GPSUtil* getInstance();
    private:
        GPSUtil();
        GPSUtil(const GPSUtil&) = delete;
        GPSUtil& operator=(const GPSUtil&) = delete;
        static GPSUtil* pInstance;
        // GPS related variables
        TinyGPSPlus gps;
        HardwareSerial serial;
        // GPS constants | GPS TX - PIN 4 | GPS RX - PIN 5
        // changed gps tx pin from 4 to 10 due to mega2560 limitations for rx signal 
        static const uint8_t GPS_RX_PIN = 10, GPS_TX_PIN = 5;
        static const uint32_t baudRate = 9600;
};

#endif