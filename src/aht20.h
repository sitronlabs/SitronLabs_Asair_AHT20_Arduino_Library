#ifndef AHT20_H
#define AHT20_H

/* Arduino libraries */
#include <Arduino.h>
#include <Wire.h>

/* C liraries */
#include <errno.h>

class aht20 {

   public:
    void setup(TwoWire &i2c_library = Wire);
    bool detect(const bool wait = true);
    int measurement_async_trigger(void);
    int measurement_async_get(float &temperature_c, float &humidity_pc);
    int measurement_sync_get(float &temperature_c, float &humidity_pc);
    int data_read(uint8_t *const bytes, const size_t length);
    int data_write(const uint8_t *const bytes, const size_t length);

   protected:
    TwoWire *m_i2c_library = NULL;
};

#endif
