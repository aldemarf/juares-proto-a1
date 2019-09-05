#include "GPSUtil.h"

// global static pointer used to ensure a single instance of the class.
GPSUtil* GPSUtil::pInstance = nullptr;

/*****************************************************************
This function is called to create an instance of the class.
Calling the constructor publicly is not allowed. The constructor
is private and is only called by this getInstance() function.
*****************************************************************/
GPSUtil* GPSUtil::getInstance() {
    if (!pInstance)   // Only allow one instance of class to be generated.
        pInstance = new GPSUtil();
    return pInstance;
}

GPSUtil::GPSUtil() : gps(), serial(HardwareSerial(1)) ();

char[] GPSUtil::getLocation() {
    char sample_str[100];
    // init string with a empty value
    sprintf(sample_str, "");
    smartDelay(1);
    if (gps.location.isUpdated() && gps.location.isValid()) {
        setTime(gps.time.hour(), gps.time.minute(), gps.time.second(),
                gps.date.day(), gps.date.month(), gps.date.year());
        char lat_str[20];
        char lng_str[20];
        dtostrf(gps.location.lat(), 4, 6, lat_str);
        dtostrf(gps.location.lng(), 4, 6, lng_str);
        sprintf(sample_str, "%lu;%s;%s\n", now(), lat_str, lng_str);
    }
    return sample_str;
}

static void GPSUtil::smartDelay(unsigned long ms)
{
    unsigned long start = millis();
    do
    {
        while (GPSSerial.available())
        {
            gps.encode(GPSSerial.read());
        }
        status_led.Update();
    } while (millis() - start < ms);
}