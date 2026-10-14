#include "laser.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "vl53l0x_api.h"
#include "vl53l0x_idf.h"

static const char *TAG = "VL53L0X";
static VL53L0X_Dev_t sensor; // estructura de ST: estado/calibracion del sensor
static bool inicializado = false;

static esp_err_t comprobar_st(VL53L0X_Error error, const char *operacion)
{
    if (error == VL53L0X_ERROR_NONE) return ESP_OK;
    ESP_LOGE(TAG, "%s fallo. Codigo ST = %d", operacion, (int)error);
    return ESP_FAIL;
}

esp_err_t laser_init(void)
{
    // Implementacion de I2C del componente externo (API de ST adaptada a ESP-IDF).
    const vl53l0x_idf_config_t configuracion = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = LASER_SDA_GPIO,
        .scl_io_num = LASER_SCL_GPIO,
        .scl_speed_hz = 100000,      // 100 kHz para primera prueba; admite hasta 400 kHz
        .i2c_address_7bit = LASER_I2C_ADDR,
        .enable_internal_pullup = 1, // NO reemplaza los pull-ups externos del bus
        .xshut_gpio_num = -1,        // modulo con XSHUT ya habilitado por su placa
        .gpio1_gpio_num = -1,        // por ahora polling (sin interrupcion externa)
        .existing_bus = NULL,
    };

    esp_err_t err = vl53l0x_idf_init(&sensor, &configuracion);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No responde sensor en 0x29 o no se pudo iniciar I2C: %s",
                 esp_err_to_name(err));
        return err;
    }

    // Secuencia de inicializacion de la API oficial ST (UM2039).
    if (comprobar_st(VL53L0X_DataInit(&sensor), "DataInit") != ESP_OK) return ESP_FAIL;
    if (comprobar_st(VL53L0X_StaticInit(&sensor), "StaticInit") != ESP_OK) return ESP_FAIL;

    uint32_t numero_spads = 0;
    uint8_t apertura_spads = 0;
    if (comprobar_st(VL53L0X_PerformRefSpadManagement(
            &sensor, &numero_spads, &apertura_spads), "RefSpadManagement") != ESP_OK)
        return ESP_FAIL;

    uint8_t vhv = 0;
    uint8_t fase = 0;
    if (comprobar_st(VL53L0X_PerformRefCalibration(
            &sensor, &vhv, &fase), "RefCalibration") != ESP_OK)
        return ESP_FAIL;

    if (comprobar_st(VL53L0X_SetDeviceMode(
            &sensor, VL53L0X_DEVICEMODE_SINGLE_RANGING), "SetDeviceMode") != ESP_OK)
        return ESP_FAIL;

    inicializado = true;
    ESP_LOGI(TAG, "Sensor inicializado, I2C 0x29 (7-bit), SDA=%d, SCL=%d", LASER_SDA_GPIO, LASER_SCL_GPIO);
    return ESP_OK;
}

esp_err_t laser_medicion(float *metros, uint16_t *milimetros, uint8_t *estado_st)
{
    if (!inicializado || !metros || !milimetros || !estado_st) return ESP_ERR_INVALID_STATE;
    *metros = 0.0f;
    *milimetros = 0;
    *estado_st = 255;

    VL53L0X_RangingMeasurementData_t dato = {0};
    VL53L0X_Error error = VL53L0X_PerformSingleRangingMeasurement(&sensor, &dato);
    if (comprobar_st(error, "PerformSingleRangingMeasurement") != ESP_OK) return ESP_FAIL;

    // IMPORTANTISIMO: distancia en milimetros no implica que la medicion sea valida.
    *estado_st = dato.RangeStatus;
    if (dato.RangeStatus != 0) {
        ESP_LOGW(TAG, "Ranging no valido: RangeStatus=%u", (unsigned)dato.RangeStatus);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (dato.RangeMilliMeter == 0 || dato.RangeMilliMeter > 2000) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *milimetros = (uint16_t)dato.RangeMilliMeter;
    *metros = dato.RangeMilliMeter / 1000.0f;
    return ESP_OK;
}
