/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

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

//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18   ,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = {26, 3, 4},
};

void LoRaUtil::send(char* data) {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        ESP_LOGE(tag, "OP_TXRXPEND, not sending");
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, data, sizeof(data)-1, 0);
        ESP_LOGI(tag, "Packet queued");
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void LoRaUtil::setup() {
    ESP_LOGI(tag, "Starting LoRa!");
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // Start job (sending automatically starts OTAA too)
    send("testando");
}

void LoRaUtil::loop() {
    os_runloop_once();
}