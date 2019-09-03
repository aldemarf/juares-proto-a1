#include "MPUUtil.h"

// global static pointer used to ensure a single instance of the class.
MPUUtil* MPUUtil::pInstance = nullptr;

/*****************************************************************
This function is called to create an instance of the class.
Calling the constructor publicly is not allowed. The constructor
is private and is only called by this getInstance() function.
*****************************************************************/
MPUUtil* MPUUtil::getInstance() {
    if (!pInstance)   // Only allow one instance of class to be generated.
        pInstance = new MPUUtil();
    return pInstance;
}

MPUUtil::MPUUtil() {
    sd = SDUtil::getInstance();
}

void MPUUtil::write_data(char* filename) {
  // sprintf(filename, "/%lu-mpu.txt", start_ts);
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

void MPUUtil::read() {
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

void MPUUtil::setup() {

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

3

}