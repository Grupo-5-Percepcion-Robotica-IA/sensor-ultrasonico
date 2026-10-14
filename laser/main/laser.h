#ifndef LASER_VL53L0X_H
#define LASER_VL53L0X_H

#include <stdint.h>
#include "esp_err.h"

// ESP32 WROOM-32: bus I2C dedicado al VL53L0X.
#define LASER_SDA_GPIO 21
#define LASER_SCL_GPIO 22
#define LASER_I2C_ADDR 0x29

esp_err_t laser_init(void);

// Mide una vez. Devuelve ESP_OK SOLO si la API informa RangeStatus=0.
// metros: distancia en m; milimetros: dato original; estado_st: diagnostico.
esp_err_t laser_medicion(float *metros, uint16_t *milimetros, uint8_t *estado_st);

#endif
