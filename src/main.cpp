#include <Arduino.h>

#include <TinyGPS++.h>
#include <TimeLib.h>
#include <jled.h>
#include <TimeAlarms.h>
// #include <WiFi.h>
#include "SDUtil.h"
// #include "MPUUtil.h"
#include "GPSUtil.h"

const uint8_t statusLED_PIN = 14;
static const uint32_t uS_TO_mS_FACTOR = 1000; /* Conversion factor for micro seconds to seconds */
static const uint16_t TIME_TO_SLEEP = 1000;   /* Time ESP32 will go to sleep (in miliseconds) */

#define DEBUG // TODO: remove this when start using ESPLOG

// SD card control object
SDUtil *sd = SDUtil::getInstance();
// MPU-6050 control object
// MPUUtil *mpu = MPUUtil::getInstance();
// GPS control object
GPSUtil *gps = GPSUtil::getInstance();
// status LED configuration
auto statusLED = JLed(statusLED_PIN);
// tag for logging system info
static const char *tag = "juares";
// variable to store the reason of device restart
esp_reset_reason_t rst_reason;
// variable that stores the time stamp of system start
RTC_DATA_ATTR time_t startTS = 0;

// application constants
const uint16_t GPS_READ_PERIOD_S = 5;

#if 0
// sleep mode constants
RTC_DATA_ATTR bool initialBoot = true;


const uint16_t MPU_READ_PERIOD_MS = 2000;
const uint8_t NUM_SAMPLES = 100;


// application variables
unsigned long time_now = 0;
auto statusLED = JLed(statusLED_PIN);
RTC_DATA_ATTR bool dmp_ready = false;  // set true if DMP init was successful
uint8_t mpu_int_status;   // holds actual interrupt status byte from MPU
uint8_t dev_status;      // return status after each device operation (0 = success, !0 = error)
RTC_DATA_ATTR uint16_t packet_size;    // expected DMP packet size (default is 42 bytes)
uint8_t fifo_buffer[64]; // FIFO storage buffer

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpu_interrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpu_interrupt = true;
}

#endif

void readGPS() {
  char strBuffer[50];
  char filename[20];

  sprintf(filename, "/%lu-gps.txt", startTS);
  gps->getLocation(strBuffer);
  sd->appendFile(filename, strBuffer);
  statusLED.Blink(250, 250).Repeat(2);
}

void setup()
{
  rst_reason = esp_reset_reason();
  Serial.begin(115200);
  gps->setup();
  sd->setup();
  // program periodical functions for GPS reading
  Alarm.timerRepeat(GPS_READ_PERIOD_S, readGPS);   
  // configure the sleep timer for the system
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_mS_FACTOR);
  if (rst_reason == ESP_RST_POWERON)
  {
    // mpu->setup();
    // signal that GPS is waiting to fix
    ESP_LOGI(tag, "Waiting for GPS fix...");
    statusLED.Blink(250, 250).Forever();
    while (!gps->isFixed())
    {
      statusLED.Update();
    };
    ESP_LOGI(tag, "GPS fixed.");
    // update system data from GPS
    gps->updateSystemTime();
    startTS = now();
    ESP_LOGI("System first boot.");
  }
  else
  {
    // mpu->wakeup();
  }
}
// TODO: trabalhar no loop!!!
void loop()
{
  Alarm.delay(1);
  statusLED.Update();
#if 0
  if (!dmp_ready)
    return;
  // verifies GPS read period to write data on file
  if (millis() > time_now + GPS_READ_PERIOD_MS)
  {
    time_now = millis();
    read_gps();
  }
  // smartDelay(MPU_READ_PERIOD_MS);
  // reads the MPU data
  read_mpu();

  // if buffer full
  if (cur_sample >= (0.8 * NUM_SAMPLES))
  {
    ESP_LOGI(tag, "%d", cur_sample);
    write_mpu_data();
    // reset buffer index
    cur_sample = 0;
  }
  else
  {
    Serial.println("Going to sleep now");
    Serial.flush();
    esp_deep_sleep_start();
  }
#endif
}