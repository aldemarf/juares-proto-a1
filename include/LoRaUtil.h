#ifndef __LORAUTIL_H__
#define __LORAUTIL_H__

class LoRaUtil {
    public:
        static LoRaUtil* getInstance();
    private:
        LoRaUtil();
        LoRaUtil(const LoRaUtil&) = delete;
        LoRaUtil& operator=(const LoRaUtil&) = delete;
        static LoRaUtil* pInstance;
};


#endif