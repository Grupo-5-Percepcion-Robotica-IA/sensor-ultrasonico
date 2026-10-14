#include "ultrasonido.h"

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_types.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define RMT_HZ 1000000                  // 1 tick = 1 microsegundo (TX y RX)
#define ECO_MAX_NS 26000000            // 26 ms: fin de la captura tras inactividad
#define ESPERA_RX_MS 60                // timeout esperando una respuesta (NO es el periodo)
#define MAX_SIMBOLOS 64
#define DURACION_TRIG_US 12            // el datasheet pide por lo menos 10 us

// Dos canales distintos del periferico RMT:
// TX genera TRIG en GPIO23; RX mide ECHO en GPIO18.
static rmt_channel_handle_t tx_chan = NULL;
static rmt_channel_handle_t rx_chan = NULL;
static rmt_encoder_handle_t tx_encoder = NULL;
static QueueHandle_t cola_rx = NULL;
static rmt_symbol_word_t rx_buffer[MAX_SIMBOLOS];

// El encoder de copia transforma este simbolo directamente en la forma de onda TRIG.
// 12 ticks en alto + 2 ticks en bajo = pulso TRIG de 12 us.
static const rmt_symbol_word_t pulso_trig = {
    .level0 = 1,
    .duration0 = DURACION_TRIG_US,
    .level1 = 0,
    .duration1 = 2,
};

typedef struct {
    uint32_t alto_us;
} evento_rx_t;

static bool rx_terminado(rmt_channel_handle_t canal,
                         const rmt_rx_done_event_data_t *edata, void *contexto)
{
    (void)canal;
    QueueHandle_t cola = (QueueHandle_t)contexto;
    evento_rx_t evento = {.alto_us = 0};

    // La duracion en alto de ECHO es proporcional a la distancia.
    // Un simbolo RMT incluye dos segmentos (nivel0/duracion0, nivel1/duracion1).
    for (size_t i = 0; i < edata->num_symbols; ++i) {
        rmt_symbol_word_t s = edata->received_symbols[i];
        if (s.level0 == 1 && s.duration0 >= 100 && s.duration0 <= 24000) {
            evento.alto_us = s.duration0;
            break;
        }
        if (s.level1 == 1 && s.duration1 >= 100 && s.duration1 <= 24000) {
            evento.alto_us = s.duration1;
            break;
        }
    }

    // Enviamos el tiempo a la tarea normal: no calculamos distancia en la ISR.
    BaseType_t despertar = pdFALSE;
    xQueueSendFromISR(cola, &evento, &despertar);
    return despertar == pdTRUE;
}

esp_err_t ultrasonido_init(void)
{
    cola_rx = xQueueCreate(1, sizeof(evento_rx_t));
    if (!cola_rx) return ESP_ERR_NO_MEM;

    // Canal RX: recibe la senal ECHO por GPIO18.
    rmt_rx_channel_config_t config_rx = {
        .gpio_num = US_ECHO_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_HZ,
        .mem_block_symbols = MAX_SIMBOLOS,
        .flags.invert_in = false,
        .flags.with_dma = false,
    };
    ESP_RETURN_ON_ERROR(rmt_new_rx_channel(&config_rx, &rx_chan), "HC_SR04", "crear RMT RX");

    rmt_rx_event_callbacks_t callbacks = {.on_recv_done = rx_terminado};
    ESP_RETURN_ON_ERROR(rmt_rx_register_event_callbacks(rx_chan, &callbacks, cola_rx),
                        "HC_SR04", "registrar callback RX");

    // Canal TX: genera TRIG por GPIO23 con tiempos definidos por hardware.
    // Ya no usamos gpio_set_level() ni esp_rom_delay_us().
    rmt_tx_channel_config_t config_tx = {
        .gpio_num = US_TRIG_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
        .flags.invert_out = false,
        .flags.with_dma = false,
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&config_tx, &tx_chan), "HC_SR04", "crear RMT TX");

    // El copy encoder envia literalmente el simbolo alto+bajo que preparamos.
    rmt_copy_encoder_config_t config_encoder = {};
    ESP_RETURN_ON_ERROR(rmt_new_copy_encoder(&config_encoder, &tx_encoder),
                        "HC_SR04", "crear encoder TX");

    ESP_RETURN_ON_ERROR(rmt_enable(rx_chan), "HC_SR04", "habilitar RMT RX");
    ESP_RETURN_ON_ERROR(rmt_enable(tx_chan), "HC_SR04", "habilitar RMT TX");
    return ESP_OK;
}

esp_err_t ultrasonido_medicion(float *distancia_m, uint32_t *pulso_us)
{
    if (!distancia_m || !pulso_us || !rx_chan || !tx_chan || !tx_encoder)
        return ESP_ERR_INVALID_ARG;

    *distancia_m = 0.0f;
    *pulso_us = 0;
    xQueueReset(cola_rx);

    // Primero empezamos a escuchar ECHO (RX), para no perder el pulso.
    const rmt_receive_config_t config_rx = {
        .signal_range_min_ns = 1500,
        .signal_range_max_ns = ECO_MAX_NS,
    };
    esp_err_t err = rmt_receive(rx_chan, rx_buffer, sizeof(rx_buffer), &config_rx);
    if (err != ESP_OK) return err;

    // Luego transmitimos TRIG por el segundo canal RMT.
    const rmt_transmit_config_t config_tx = {
        .loop_count = 0,
        .flags.eot_level = 0,
    };
    err = rmt_transmit(tx_chan, tx_encoder, &pulso_trig, sizeof(pulso_trig), &config_tx);
    if (err == ESP_OK) {
        // rmt_transmit() encola; esto asegura que el pulso haya terminado.
        err = rmt_tx_wait_all_done(tx_chan, 20);  // esta funcion usa milisegundos
    }
    if (err != ESP_OK) {
        // Si TX falla, no dejar RX escuchando una captura vieja.
        rmt_disable(rx_chan);
        rmt_enable(rx_chan);
        return err;
    }

    evento_rx_t evento = {0};
    bool recibido = xQueueReceive(cola_rx, &evento, pdMS_TO_TICKS(ESPERA_RX_MS)) == pdTRUE;

    // Dejamos RX limpio para la siguiente medicion; TX permanece habilitado.
    ESP_RETURN_ON_ERROR(rmt_disable(rx_chan), "HC_SR04", "detener RMT RX");
    ESP_RETURN_ON_ERROR(rmt_enable(rx_chan), "HC_SR04", "reanudar RMT RX");

    if (!recibido) return ESP_ERR_TIMEOUT;
    if (evento.alto_us == 0) return ESP_ERR_INVALID_RESPONSE;

    // Distancia = (343 m/s * duracion_ECHO_en_segundos) / 2.
    float metros = evento.alto_us * 0.0001715f;
    if (metros < 0.02f || metros > 4.0f) return ESP_ERR_INVALID_RESPONSE;

    *distancia_m = metros;
    *pulso_us = evento.alto_us;
    return ESP_OK;
}