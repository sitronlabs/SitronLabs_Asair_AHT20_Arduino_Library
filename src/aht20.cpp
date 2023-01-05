/* Self header */
#include "aht20.h"

/* Fixed configuration values */
#define CONFIG_AHT20_I2C_ADDRESS 0x38         //!< Address of the sensor on the i2c bus, in the 7-bit notation.
#define CONFIG_AHT20_CALIBRATION_DURATION 10  //!< Amount of time needed by the sensor to perform the internal calibration when requested. Value in milliseconds, taken from the datasheet.
#define CONFIG_AHT20_MEASUREMENT_DURATION 80  //!< Amount of time needed by the sensor to perform a measure when requested. Value in milliseconds, taken from the datasheet.

/* Overriable configuration values */
#ifndef CONFIG_AHT20_MEASUREMENT_RETRY_DELAY
#define CONFIG_AHT20_MEASUREMENT_RETRY_DELAY 1  //!< Amount of time the library should wait before checking if the measurement is done. Value in milliseconds.
#endif
#ifndef CONFIG_AHT20_DEBUG_ENABLED
#define CONFIG_AHT20_DEBUG_ENABLED 0  //!< Set this to 1 to use turn on verbose debugging.
#endif
#if CONFIG_AHT20_DEBUG_ENABLED
#ifndef CONFIG_AHT20_DEBUG_FUNCTION
#define CONFIG_AHT20_DEBUG_FUNCTION(x) Serial.println(x)
#endif
#else
#define CONFIG_AHT20_DEBUG_FUNCTION(x)
#endif

/**
 *
 * @param[in] i2c_library
 */
void aht20::setup(TwoWire &i2c_library) {
    m_i2c_library = &i2c_library;
}

/**
 * Tries to detect the sensor.
 * @note It takes a maximum of 20 ms after power up for the sensor to be ready to receive commands.
 * @param[in] wait If set to false the library will not make sure 20 ms have elapsed since boot.
 * @return true if the sensor has been detected, or false otherwise.
 */
bool aht20::detect(const bool wait) {
    int res;

    /* Wait for power up */
    if (wait) {
        while (millis() < 20)
            ;
    }

    /* Ping device */
    m_i2c_library->beginTransmission(CONFIG_AHT20_I2C_ADDRESS);
    res = m_i2c_library->endTransmission(true);
    if (res != 0) {
        return false;
    }

    /* Return success */
    return true;
}

/**
 * Request the sensor to trigger a measurement and returns immediately.
 * After that, make repeated calls to measurement_async_get until the functions return anything else than -EINPROGRESS.
 * @return 0 in case of success, or a negative error code otherwise.
 */
int aht20::measurement_async_trigger(void) {
    int res;

    /* Remember start time to detect timeout */
    uint32_t time_start = millis();

    /* Ensure calibration has been performed */
    while (1) {

        /* Read calibration status bit */
        uint8_t reg_status;
        res = data_read(&reg_status, 1);
        if (res < 1) {
            CONFIG_AHT20_DEBUG_FUNCTION(" [e] Failed to read status register!");
            return res;
        }

        /* If calibration has been done, we can stop here */
        if ((reg_status & (1 << 3))) {
            break;
        }

        /* Request to start calibration */
        uint8_t cmd[] = {0xBE, 0x08, 0x00};
        res = data_write(cmd, 3);
        if (res < 0) {
            CONFIG_AHT20_DEBUG_FUNCTION(" [e] Failed to send calibration command!");
            return res;
        }

        /* Wait enough time for calibration to be done */
        delay(CONFIG_AHT20_CALIBRATION_DURATION);

        /* Detect timeout */
        if (millis() - time_start > 2 * CONFIG_AHT20_CALIBRATION_DURATION) {
            CONFIG_AHT20_DEBUG_FUNCTION(" [e] Timed out waiting for calibration!");
            return -ETIMEDOUT;
        }
    }

    /* Request to trigger measurement */
    uint8_t cmd[] = {0xAC, 0x33, 0x00};
    res = data_write(cmd, 3);
    if (res < 0) {
        CONFIG_AHT20_DEBUG_FUNCTION(" [e] Failed to send calibration command!");
        return res;
    }

    /* Return success */
    return 0;
}

/**
 * Retrieves the measurement results if available.
 * Call measurement_async_trigger first and then call this function repeatedly as long as it returns -EINPROGRESS.
 * @param[out] temperature_c A variable that will be updated with the temperature in degrees Celsius.
 * @param[out] humidity_pc A variable that will be updated with the humidity in percents (0-100).
 * @return 0 in case of success, -EINPROGRESS if a measurement is still in progress or a negative error code otherwise.
 */
int aht20::measurement_async_get(float &temperature_c, float &humidity_pc) {
    int res;

    /* Read all registers */
    uint8_t regs[7];
    res = data_read(regs, 7);
    if (res < 0) {
        CONFIG_AHT20_DEBUG_FUNCTION(" [e] Failed to read registers!");
        return res;
    }

    /* Wait for busy bit to be cleared */
    if (regs[0] & (1 << 7)) {
        return -EINPROGRESS;
    }

    /* Extract 20-bits of humidity data
     * See Datasheet section "6.1 Relative humidity transformation" */
    uint32_t reg_humidity = regs[1];
    reg_humidity <<= 8;
    reg_humidity |= regs[2];
    reg_humidity <<= 4;
    reg_humidity |= (regs[3] >> 4);
    humidity_pc = (reg_humidity / 10485.76);

    /* Extract 20-bits of temperature data
     * See Datasheet section "6.2 Temperature transformation" */
    uint32_t reg_temperature = (regs[3] & 0x0F);
    reg_temperature <<= 8;
    reg_temperature |= regs[4];
    reg_temperature <<= 8;
    reg_temperature |= regs[5];
    temperature_c = (reg_temperature / 1048576.0) * 200.0 - 50.0;

    /* Check crc
     * Using a seed of 0xFF and a polynomial of 0x31 */
    uint8_t crc8_computed = 0xFF;
    for (size_t i = 0; i < 6; i++) {
        crc8_computed ^= regs[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc8_computed & (1 << 7)) {
                crc8_computed = (crc8_computed << 1) ^ 0x31;
            } else {
                crc8_computed = (crc8_computed << 1);
            }
        }
    }
    if (crc8_computed != regs[6]) {
        CONFIG_AHT20_DEBUG_FUNCTION(" [e] Checksum mismatch!");
        return -EIO;
    }

    /* Return success */
    return 0;
}

/**
 *
 * @param[out] temperature
 * @param[out] humidity
 * @return
 */
int aht20::measurement_sync_get(float &temperature, float &humidity) {
    int res;

    /* Trigger measurement */
    res = measurement_async_trigger();
    if (res < 0) {
        CONFIG_AHT20_DEBUG_FUNCTION(" [e] Failed to trigger measurement!");
        return res;
    }

    /* Remember start time to detect timeout */
    uint32_t time_start = millis();

    /* Wait enough time for measurement to be done and try to request results */
    delay(CONFIG_AHT20_MEASUREMENT_DURATION);
    while (1) {

        /* Detect timeout */
        if (millis() - time_start > 2 * CONFIG_AHT20_MEASUREMENT_DURATION) {
            CONFIG_AHT20_DEBUG_FUNCTION(" [e] Timed out waiting for measurement!");
            return -ETIMEDOUT;
        }

        /* Request measurement results,
         * But if they are not ready yet, wait a bit more */
        res = measurement_async_get(temperature, humidity);
        if (res == -EINPROGRESS) {
            delay(CONFIG_AHT20_MEASUREMENT_RETRY_DELAY);
            continue;
        } else if (res < 0) {
            CONFIG_AHT20_DEBUG_FUNCTION(" [e] Failed to retrieve measurement!");
            return res;
        } else if (res == 0) {
            break;
        }
    }

    /* Return success */
    return 0;
}

/**
 *
 * @param[out] bytes
 * @param[in] length
 * @return the number of bytes read from the sensor in case of success, or a negative error code otherwise.
 */
int aht20::data_read(uint8_t *const bytes, const size_t length) {
    int res;

    /* Ensure library has been set */
    if (m_i2c_library == NULL) {
        return -EINVAL;
    }

    /* Request to read */
    res = m_i2c_library->requestFrom(CONFIG_AHT20_I2C_ADDRESS, length, true);
    if (res < 0) {
        return -EIO;
    }

    /* Return bytes */
    for (size_t i = 0; i < length && i < (size_t)res; i++) {
        bytes[i] = m_i2c_library->read();
    }
    return res;
}

/**
 *
 * @param[int] bytes
 * @param[in] length
 * @return the number of bytes written to the sensor in case of success, or a negative error code otherwise.
 */
int aht20::data_write(const uint8_t *const bytes, const size_t length) {
    int res;

    /* Ensure library has been set */
    if (m_i2c_library == NULL) {
        return -EINVAL;
    }

    /* Request to write */
    m_i2c_library->beginTransmission(CONFIG_AHT20_I2C_ADDRESS);
    m_i2c_library->write(bytes, length);
    res = m_i2c_library->endTransmission(true);
    if (res != 0) {
        return -EIO;
    }

    /* Return bytes */
    return res;
}
