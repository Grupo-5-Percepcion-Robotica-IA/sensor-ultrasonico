#ifndef ULTRASONIDO_H
#define ULTRASONIDO_H

#include <stdint.h>
#include "esp_err.h"

// GPIO23 -> TRIG; GPIO18 <- ECHO (OBLIGATORIO divisor resistivo a 3.3 V)
#define US_TRIG_GPIO 23
#define US_ECHO_GPIO 18

esp_err_t ultrasonido_init(void);
// Devuelve ESP_OK y distancia en metros si hay eco válido; si no, ESP_ERR_TIMEOUT o ESP_ERR_INVALID_RESPONSE.
esp_err_t ultrasonido_medicion(float *distancia_m, uint32_t *pulso_us);

#endif
