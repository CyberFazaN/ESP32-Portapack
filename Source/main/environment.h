/*
 * Copyright (C) 2024 HTotoo
 *
 * This file is part of ESP32-Portapack.
 *
 * For additional license information, see the LICENSE file.
 */


#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <math.h>
#include "esp_log.h"
// include supported modules
#include "i2cdev.h"

#include "drivers/bmp280.h"
#include "drivers/bh1750.h"
#include "drivers/sht3x.h"

typedef enum EnvironmentSensors
{
    Environment_none = 0,
    Environment_bmp280 = 1,
    Environment_bme280 = 2,
    Environment_sht3x = 4,
} EnvironmentSensors;

typedef enum EnvironmentLightSensors
{
    Environment_light_none = 0,
    Environment_light_bh1750 = 1
} EnvironmentLightSensors;

#if __cplusplus
extern "C"
{
#endif

    void init_environment();
    void init_environment_light();

    void get_environment_meas(float *temperature, float *pressure, float *humidity);
    void get_environment_light(uint16_t *light);

    bool is_environment_sensor_present();
    bool is_environment_light_sensor_present();

#if __cplusplus
}
#endif

#endif