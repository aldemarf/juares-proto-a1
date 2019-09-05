#include <Arduino.h>

#include <TinyGPS++.h>
#include <TimeLib.h>
#include <jled.h>
#include <Wire.h>
#include <WiFi.h>
#include "SDUtil.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define DEBUG

// SD card control object
SDUtil *sd = SDUtil::getInstance();
// MPU-6050 control object
MPUUtil *mpu = MPUUtil::getInstance();
// GPS control object
GPSUtil *gps = GPSUtil::getInstance();
#if 0
// sleep mode constants
static const uint32_t  uS_TO_S_FACTOR = 1000;  /* Conversion factor for micro seconds to seconds */
static const uint16_t  TIME_TO_SLEEP = 1000;        /* Time ESP32 will go to sleep (in miliseconds) */
RTC_DATA_ATTR bool initialBoot = true;

// application constants
const uint16_t GPS_READ_PERIOD_MS = 60000;
const uint16_t MPU_READ_PERIOD_MS = 2000;
const uint8_t NUM_SAMPLES = 100;
const uint8_t STATUS_LED_PIN = 14;

// application variables
unsigned long time_now = 0;
auto status_led = JLed(STATUS_LED_PIN);
RTC_DATA_ATTR bool dmp_ready = false;  // set true if DMP init was successful
uint8_t mpu_int_status;   // holds actual interrupt status byte from MPU
uint8_t dev_status;      // return status after each device operation (0 = success, !0 = error)
RTC_DATA_ATTR uint16_t packet_size;    // expected DMP packet size (default is 42 bytes)
uint8_t fifo_buffer[64]; // FIFO storage buffer
static const char* tag = "jua-ams";
 esp_reset_reason_t rst_reason;

RTC_DATA_ATTR time_t start_ts = 0;
char filename[30];

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpu_interrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpu_interrupt = true;
}

#endif

void setup() {
    rst_reason = esp_reset_reason();
    Serial.begin(115200);
    // init GPS serial interface
    GPSSerial.begin(9600, SERIAL_8N1, 12, 15);
    // init MPU-6050
    mpu_setup();
    sd->setup();
  if(rst_reason == ESP_RST_POWERON) {
    // wait for GPS fix
    status_led.Blink(250, 250).Forever();
    ESP_LOGI(tag, "waiting gps fix...");
    // Serial.print(F("\nwaiting gps fix: "));
    do {
        status_led.Update();
        smartDelay(250);
    } while (!gps.location.isValid() || !gps.location.isUpdated() || 
             !gps.time.isValid() || !gps.time.isUpdated() ||
             !gps.date.isValid() || !gps.date.isUpdated());
    ESP_LOGI(tag, "gps fixed!");
    // Serial.println(F("ok"));
      // get time from GPS
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(),
            gps.date.day(), gps.date.month(), gps.date.year());
    start_ts = now();
    Serial.println("first boot!");
  }
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}

void loop() {
  if (!dmp_ready) return;
  // verifies GPS read period to write data on file
  if (millis() > time_now + GPS_READ_PERIOD_MS) {
    time_now = millis();
    read_gps();
  }
  // smartDelay(MPU_READ_PERIOD_MS);
  // reads the MPU data
  read_mpu();

      sprintf(filename, "/%lu-gps.txt", start_ts);
    // write GPS data on file
    sd->appendFile(SD, filename, sample_str);
    status_led.Blink(250, 250).Repeat(2);

    // if buffer full
    if (cur_sample >= (0.8 * NUM_SAMPLES)) {
        ESP_LOGI(tag, "%d", cur_sample);
      write_mpu_data();
      // reset buffer index
      cur_sample = 0;
    }
    else {
        Serial.println("Going to sleep now");
        Serial.flush(); 
        esp_deep_sleep_start();
    }
}