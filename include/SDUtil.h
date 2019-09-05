#ifndef __SDUTIL_H__
#define __SDUTIL_H__

#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

class SDUtil {
    public:
        static SDUtil* getInstance();
        void setup();
        void appendFile(const char *path, const char *message);
    private:
        SDUtil();
        SDUtil(const SDUtil&) = delete;
        SDUtil& operator=(const SDUtil&) = delete;
        static SDUtil* pInstance;
        // SD card constants
        static const uint8_t SCLK_PIN = 25; 
        static const uint8_t MISO_PIN = 32;
        static const uint8_t MOSI_PIN = 13;
        static const uint8_t SS_PIN = 33;
        // SD card related variables
        SPIClass *hspi = NULL;
        void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
        void createDir(fs::FS &fs, const char *path);
        void removeDir(fs::FS &fs, const char *path);
        void readFile(fs::FS &fs, const char *path);
        void writeFile(fs::FS &fs, const char *path, const char *message);
        void appendFile(fs::FS &fs, const char *path, const char *message);
        void renameFile(fs::FS &fs, const char *path1, const char *path2);
        void deleteFile(fs::FS &fs, const char *path);
        void testFileIO(fs::FS &fs, const char *path);
};


#endif