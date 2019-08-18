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

// MPU-6050 constants
static const uint8_t INTERRUPT_PIN = 2;

// MPU-6050 related variables
MPU6050 mpu;

// GPS constants | GPS TX - PIN 4 | GPS RX - PIN 5
// changed gps tx pin from 4 to 10 due to mega2560 limitations for rx signal 
static const uint8_t GPS_RX_PIN = 10, GPS_TX_PIN = 5;
static const uint32_t GPSBaud = 9600;
// GPS related variables
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);

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
uint16_t fifo_count;     // count of all bytes currently in FIFO
uint8_t fifo_buffer[64]; // FIFO storage buffer
static const char* tag = "jua-ams";
 esp_reset_reason_t rst_reason;

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
RTC_DATA_ATTR time_t start_ts = 0;
char filename[30];

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpu_interrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpu_interrupt = true;
}

static void smartDelay(unsigned long ms)
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

void read_gps() {
  smartDelay(1);
  if (gps.location.isUpdated() && gps.location.isValid()) {
    sprintf(filename, "/%lu-gps.txt", start_ts);
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(),
            gps.date.day(), gps.date.month(), gps.date.year());
    char sample_str[100];
    char lat_str[20];
    char lng_str[20];
    dtostrf(gps.location.lat(), 4, 6, lat_str);
    dtostrf(gps.location.lng(), 4, 6, lng_str);
    sprintf(sample_str, "%lu;%s;%s\n",
            now(), lat_str, lng_str);
#ifdef DEBUG
    Serial.print(F("\ntem localizacao... "));
    Serial.println(sample_str);
#endif
    // write GPS data on file
    sd->appendFile(SD, filename, sample_str);
    status_led.Blink(250, 250).Repeat(2);
  }
  status_led.Blink(500, 500).Forever();
}

void write_mpu_data() {
  sprintf(filename, "/%lu-mpu.txt", start_ts);
#ifdef DEBUG
  Serial.println(F("writing accelerometer data..."));
#endif
  char sample_str[200];
  char qw_str[20];
  char qx_str[20];
  char qy_str[20];
  char qz_str[20];
  // escreve todas as amostras coletadas
  for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
    dtostrf(mpu_samples[i].q.w, 4, 6, qw_str);
    dtostrf(mpu_samples[i].q.x, 4, 6, qx_str);
    dtostrf(mpu_samples[i].q.y, 4, 6, qy_str);
    dtostrf(mpu_samples[i].q.z, 4, 6, qz_str);
    sprintf(sample_str, "%lu;%s;%s;%s;%s;%d;%d;%d;%d;%d;%d\n",
            mpu_samples[i].ts, qw_str, qx_str, qy_str, qz_str,
            mpu_samples[i].gX, mpu_samples[i].gY, mpu_samples[i].gZ,
            mpu_samples[i].aX, mpu_samples[i].aY, mpu_samples[i].aZ);
    // escreve valor 'x' do acelerometro no arquivo
    sd->appendFile(SD, filename, sample_str);
#ifdef DEBUG
    if(i == 0) Serial.println(mpu_samples[0].ts);
    // Serial.println(sample_str);
#endif
  }
#ifdef DEBUG
  Serial.println(F("ok"));
#endif
}

void read_mpu() {
  fifo_count = mpu.getFIFOCount();
#ifdef DEBUG
  Serial.print("ps: "); Serial.print(packet_size);
  Serial.print(" | fc: "); Serial.println(fifo_count);
#endif
  while(fifo_count >= packet_size) {
    fifo_count -= packet_size;
    mpu_samples[cur_sample].ts = now();
    mpu.dmpGetQuaternion(&mpu_samples[cur_sample].q, fifo_buffer);
    mpu_samples[cur_sample].gX = (fifo_buffer[16] << 8) | fifo_buffer[17];
    mpu_samples[cur_sample].gY = (fifo_buffer[20] << 8) | fifo_buffer[21];
    mpu_samples[cur_sample].gZ = (fifo_buffer[24] << 8) | fifo_buffer[25];
    mpu_samples[cur_sample].aX = (fifo_buffer[28] << 8) | fifo_buffer[29];
    mpu_samples[cur_sample].aY = (fifo_buffer[32] << 8) | fifo_buffer[33];
    mpu_samples[cur_sample].aZ = (fifo_buffer[36] << 8) | fifo_buffer[37];
    cur_sample++;
    // if buffer full
    if (cur_sample == NUM_SAMPLES) {
      write_mpu_data();
      // reset buffer index
      cur_sample = 0;
    }
  }
}

void mpu_setup() {

    Wire.begin();
    Wire.setClock(400000);
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

#if 0
    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
#endif
    if(rst_reason == ESP_RST_POWERON) {
        // load and configure the DMP
        Serial.println(F("Initializing DMP..."));
        dev_status = mpu.dmpInitialize();

        // supply your own gyro offsets here, scaled for min sensitivity
        mpu.setXGyroOffset(220);
        mpu.setYGyroOffset(76);
        mpu.setZGyroOffset(-85);
        mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

        // make sure it worked (returns 0 if so)
        if (dev_status == 0) {
            // Calibration Time: generate offsets and calibrate our MPU6050
            mpu.CalibrateAccel(6);
            mpu.CalibrateGyro(6);
            mpu.PrintActiveOffsets();
            // turn on the DMP, now that it's ready
            Serial.println(F("Enabling DMP..."));
            mpu.setDMPEnabled(true);

            // enable Arduino interrupt detection
            Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
            Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
            Serial.println(F(")..."));
            attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
            mpu_int_status = mpu.getIntStatus();

            // set our DMP Ready flag so the main loop() function knows it's okay to use it
            Serial.println(F("DMP ready! Waiting for first interrupt..."));
            dmp_ready = true;

            // get expected DMP packet size for later comparison
            packet_size = mpu.dmpGetFIFOPacketSize();
        } else {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(dev_status);
            Serial.println(F(")"));
        }
    }
}

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