#include "GPSUtil.h"

// global static pointer used to ensure a single instance of the class.
GPSUtil *GPSUtil::pInstance = nullptr;

/*****************************************************************
This function is called to create an instance of the class.
Calling the constructor publicly is not allowed. The constructor
is private and is only called by this getInstance() function.
*****************************************************************/
GPSUtil *GPSUtil::getInstance()
{
    if (!pInstance) // Only allow one instance of class to be generated.
        pInstance = new GPSUtil();
    return pInstance;
}

GPSUtil::GPSUtil() : gps(), serial(HardwareSerial(1)){};

void GPSUtil::setup()
{
    // init GPS serial interface
    serial.begin(GPSUtil::baudRate, SERIAL_8N1, GPSUtil::GPS_RX_PIN, GPSUtil::GPS_TX_PIN);
}

bool GPSUtil::getLocation(char *locationStr)
{
    bool successFlag = false;
    readSerial(1);
    if (gps.location.isUpdated() && gps.location.isValid())
    {
        // refresh system time
        updateSystemTime();
        char lat_str[20];
        char lng_str[20];
        dtostrf(gps.location.lat(), 4, 6, lat_str);
        dtostrf(gps.location.lng(), 4, 6, lng_str);
        sprintf(locationStr, "%lu;%s;%s\n", now(), lat_str, lng_str);
        successFlag == true;
    }
    return successFlag;
}

bool GPSUtil::isFixed()
{
    bool fixedFlag = false;
    readSerial(1);
    if (gps.location.isValid() && gps.location.isUpdated() &&
        gps.time.isValid() && gps.time.isUpdated() &&
        gps.date.isValid() && gps.date.isUpdated())
    {
        fixedFlag = true;
    }
    return fixedFlag;
}

void GPSUtil::updateSystemTime()
{
    // set the system time from GPS
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(),
            gps.date.day(), gps.date.month(), gps.date.year());
}

// TODO: consider removing the timeout as it doesnt make sense for the new approach
void GPSUtil::readSerial(unsigned long timeout_ms)
{
    unsigned long start = millis();
    do
    {
        while (serial.available())
        {
            gps.encode(serial.read());
        }
    } while (millis() - start < timeout_ms);
}