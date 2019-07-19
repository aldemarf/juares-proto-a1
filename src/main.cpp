#include <Arduino.h>

/*
 * Connect the SD card to the following pins:
 *
 * SD Card | ESP32
 *    D2       -
 *    D3       SS
 *    CMD      MOSI
 *    VSS      GND
 *    VDD      3.3V
 *    CLK      SCK
 *    VSS      GND
 *    D0       MISO
 *    D1       -
 */
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <TimeLib.h>
#include <jled.h>
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#undef DEBUG

#define SCLK_PIN 25
#define MISO_PIN 32
#define MOSI_PIN 13
#define SS_PIN 33

// const int MPU = 0x68;
MPU6050 mpu;
const uint16_t periodMPU = 100;
const uint16_t num_accel_samples = 10;
#define MPU_INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

#define BAT_MEASURE_ADC 35 // battery probe GPIO pin -> ADC1_CHANNEL_7
#define BAT_VOLTAGE_DIVIDER 2

unsigned long time_now = 0;

struct acc_samples_t
{
    uint32_t ts;
    uint16_t voltage;
    int32_t AcX;
    int32_t AcY;
    int32_t AcZ;
    int32_t Tmp;
    int32_t GyX;
    int32_t GyY;
    int32_t GyZ;
} acc_samples[num_accel_samples];

SPIClass *hspi = NULL;
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);
char gpsFileName[80];
char accFileName[80];
const uint16_t period = 60000;
uint32_t start_ts;
uint16_t cur_sample = 0;
auto led = JLed(14);

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
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
        led.Update();
    } while (millis() - start < ms);
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if (!root)
    {
        Serial.println("Failed to open directory");
        return;
    }
    if (!root.isDirectory())
    {
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file)
    {
        if (file.isDirectory())
        {
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if (levels)
            {
                listDir(fs, file.name(), levels - 1);
            }
        }
        else
        {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char *path)
{
    Serial.printf("Creating Dir: %s\n", path);
    if (fs.mkdir(path))
    {
        Serial.println("Dir created");
    }
    else
    {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char *path)
{
    Serial.printf("Removing Dir: %s\n", path);
    if (fs.rmdir(path))
    {
        Serial.println("Dir removed");
    }
    else
    {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char *path)
{
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if (!file)
    {
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while (file.available())
    {
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to open file for writing");
        return;
    }
    if (file.print(message))
    {
        Serial.println("File written");
    }
    else
    {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if (!file)
    {
        Serial.println("Failed to open file for appending");
        led.Blink(100, 100).Forever();
        return;
    }
    if (file.print(message))
    {
        Serial.println("Message appended");
    }
    else
    {
        led.Blink(100, 100).Forever();
        Serial.println("Append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2)
{
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2))
    {
        Serial.println("File renamed");
    }
    else
    {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char *path)
{
    Serial.printf("Deleting file: %s\n", path);
    if (fs.remove(path))
    {
        Serial.println("File deleted");
    }
    else
    {
        Serial.println("Delete failed");
    }
}

void testFileIO(fs::FS &fs, const char *path)
{
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if (file)
    {
        len = file.size();
        size_t flen = len;
        start = millis();
        while (len)
        {
            size_t toRead = len;
            if (toRead > 512)
            {
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    }
    else
    {
        Serial.println("Failed to open file for reading");
    }

    file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for (i = 0; i < 2048; i++)
    {
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}

void readGPS()
{
    char msg[80];
    uint32_t ts;
    double lat, lng;

    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(),
            gps.date.day(), gps.date.month(), gps.date.year());
    ts = now();
    lat = gps.location.lat();
    lng = gps.location.lng();
    sprintf(msg, "%d;%f;%f\n", ts, lat, lng);
#ifdef DEBUG
    Serial.print(ts);
    Serial.print(";");
    Serial.print(gps.location.lat(), 6);
    Serial.print(";");
    Serial.println(gps.location.lng(), 6);
#endif
    appendFile(SD, gpsFileName, msg);
}

void writeMPUData()
{
    char msg[80];
#ifdef DEBUG
    Serial.print(F("writing accelerometer data..."));
#endif
    // escreve todas as amostras coletadas
    for (uint16_t i = 0; i < cur_sample; i++)
    {
        sprintf(msg, "%d;%d;%d;%d;%d;%f;%d;%d;%d\n",
                acc_samples[i].ts,
                acc_samples[i].voltage,
                acc_samples[i].AcX,
                acc_samples[i].AcY,
                acc_samples[i].AcZ,
                ((acc_samples[i].Tmp / 340.00) + 36.53),
                acc_samples[i].GyX,
                acc_samples[i].GyY,
                acc_samples[i].GyZ);
    }
    appendFile(SD, accFileName, msg);

#ifdef DEBUG
    Serial.println(F("ok"));
#endif
}

void readMPU()
{
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    // solicita os dados do sensor
    Wire.requestFrom(MPU, 14, true);

    //Armazena o valor dos sensores nas variaveis correspondentes
    acc_samples[cur_sample].AcX = Wire.read() << 8 | Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    acc_samples[cur_sample].AcY = Wire.read() << 8 | Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    acc_samples[cur_sample].AcZ = Wire.read() << 8 | Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    acc_samples[cur_sample].Tmp = Wire.read() << 8 | Wire.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    acc_samples[cur_sample].GyX = Wire.read() << 8 | Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    acc_samples[cur_sample].GyY = Wire.read() << 8 | Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    acc_samples[cur_sample].GyZ = Wire.read() << 8 | Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    acc_samples[cur_sample].ts = now();
    acc_samples[cur_sample].voltage = analogRead(BAT_MEASURE_ADC);
#ifdef DEBUG
    Serial.print(F("millis = "));
    Serial.print(acc_samples[cur_sample].ts);
    Serial.print(F("bat_level: "));
    Serial.print(acc_samples[cur_sample].voltage);
    // envia valor 'x' do acelerometro para a serial
    Serial.print(F(" | acc_x = "));
    Serial.print(acc_samples[cur_sample].AcX);
    // envia valor 'y' do acelerometro para a serial
    Serial.print(F(" | acc_y = "));
    Serial.print(acc_samples[cur_sample].AcY);
    // envia valor 'z' do acelerometro para a serial
    Serial.print(F(" | acc_z = "));
    Serial.print(acc_samples[cur_sample].AcZ);
    // envia valor da temperatura para a serial
    Serial.print(F(" | tmp = "));
    Serial.print((acc_samples[cur_sample].Tmp / 340.00) + 36.53);
    // envia valor 'x' do giroscopio para a serial
    Serial.print(F(" | gyr_x = "));
    Serial.print(acc_samples[cur_sample].GyX);
    // envia valor 'y' do giroscopio para a serial
    Serial.print(F(" | gyr_y = "));
    Serial.print(acc_samples[cur_sample].GyY);
    // envia valor 'z' do giroscopio para a serial
    Serial.print(F(" | gyr_z = "));
    Serial.println(acc_samples[cur_sample].GyZ);
#endif
    cur_sample++;
    if (cur_sample == 10)
    {
        writeMPUData();
        // reinicializa o buffer temporario
        cur_sample = 0;
    }
}

void setup()
{
#ifdef DEBUG
    Serial.begin(115200);
#endif
    // init GPS serial interface
    GPSSerial.begin(9600, SERIAL_8N1, 12, 15);
    // init SD card SPI interface
    hspi = new SPIClass(HSPI);
    pinMode(SS_PIN, OUTPUT); //HSPI SS
    // SCLK = 25, MISO = 32, MOSI = 13, SS = 33
    hspi->begin(SCLK_PIN, MISO_PIN, MOSI_PIN, SS_PIN); //SCLK, MISO, MOSI, SS
    // init MPU on I2C
    Wire.begin();
    Wire.setClock(400000);
#ifdef DEBUG
    Serial.println(F("Initializing I2C devices..."));
#endif
    // initialize device
    mpu.initialize();
    pinMode(MPU_INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // empty buffer
    while (Serial.available() && Serial.read());

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

        // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(MPU_INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        printf("packetSize: %d", packetSize);
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    if (!SD.begin(SS_PIN, *hspi))
    {
        Serial.println("Card Mount Failed");
        led.Blink(100, 100).Forever();
        return;
    }
    uint8_t cardType = SD.cardType();

    if (cardType == CARD_NONE)
    {
        led.Blink(100, 1000).Forever();
        Serial.println("No SD card attached");
        return;
    }

    led.Blink(500, 500).Forever();
#ifdef DEBUG
    Serial.print(F("\nWaiting GPS fix... "));
#endif
    do
    {
        smartDelay(1000);
    } while (!gps.location.isValid() || !gps.location.isUpdated());

#ifdef DEBUG
    Serial.println(F("OK"));
#endif

    // captura a hora do GPS
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(),
            gps.date.day(), gps.date.month(), gps.date.year());
    start_ts = now();
    sprintf(gpsFileName, "/%d-gps.txt", start_ts);
    sprintf(accFileName, "/%d-acc.txt", start_ts);
#ifdef DEBUG
    Serial.print(F("Current TimeStamp:"));
    Serial.println(start_ts);
#endif
    led.Blink(100, 10000).Forever();
}

void loop()
{
    if (millis() > time_now + period)
    {
        time_now = millis();
        readGPS();
    }
    smartDelay(periodMPU);
    readMPU();
    if (millis() > 5000 && gps.charsProcessed() < 10)
        Serial.println(F("No GPS data received: check wiring"));
}