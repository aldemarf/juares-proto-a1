#include "LoRaUtil.h"

// global static pointer used to ensure a single instance of the class.
LoRaUtil* LoRaUtil::pInstance = nullptr;

/*****************************************************************
This function is called to create an instance of the class.
Calling the constructor publicly is not allowed. The constructor
is private and is only called by this getInstance() function.
*****************************************************************/
LoRaUtil* LoRaUtil::getInstance() {
    if (!pInstance)   // Only allow one instance of class to be generated.
        pInstance = new LoRaUtil();
    return pInstance;
}

LoRaUtil::LoRaUtil() {
    
}