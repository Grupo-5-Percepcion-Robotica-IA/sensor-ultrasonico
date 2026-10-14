#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "ultrasonido.h"
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/range.h>
#include <rosidl_runtime_c/string_functions.h>

#define ETIQUETA "ULTRASONIDO"
#define PERIODO_MEDICION_MS 70    // 70 ms entre inicios de medicion: ~14.3 Hz
#define PERIODO_PUBLICACION_MS 20 // ROS busca nuevas lecturas con mayor frecuencia

// Datos compartidos entre la tarea RMT y la tarea micro-ROS.
static portMUX_TYPE medicion_mux = portMUX_INITIALIZER_UNLOCKED;
static float ultima_distancia_m = 0.0f;
static uint32_t ultimo_pulso_us = 0;
static bool ultima_valida = false;
static uint32_t ultimo_numero = 0;

static rcl_publisher_t pub_float;
static rcl_publisher_t pub_range;
static std_msgs__msg__Float32 msg_float;
static sensor_msgs__msg__Range msg_range;

#define ROS_CHECK(x) do { \
    rcl_ret_t rc_ = (x); \
    if (rc_ != RCL_RET_OK) { \
        ESP_LOGE(ETIQUETA, "Error ROS %ld en linea %d", (long)rc_, __LINE__); \
        vTaskDelete(NULL); \
        return; \
    } \
} while (0)

static void tarea_medicion(void *arg)
{
    (void)arg;
    TickType_t proximo_despertar = xTaskGetTickCount();
    for (;;) {
        float metros = 0.0f;
        uint32_t microsegundos = 0;
        esp_err_t resultado = ultrasonido_medicion(&metros, &microsegundos);

        taskENTER_CRITICAL(&medicion_mux);
        ultima_valida = (resultado == ESP_OK);
        if (resultado == ESP_OK) {
            ultima_distancia_m = metros;
            ultimo_pulso_us = microsegundos;
        }
        ultimo_numero++;
        taskEXIT_CRITICAL(&medicion_mux);

        if (resultado == ESP_OK) {
            ESP_LOGI(ETIQUETA, "ECHO=%lu us | Distancia=%.1f cm (%.3f m)",
                     (unsigned long)microsegundos, metros * 100.0f, metros);
        } else {
            ESP_LOGW(ETIQUETA, "Sin eco valido (%s)", esp_err_to_name(resultado));
        }
        // No sumar 70 ms al tiempo que ya tardo RMT: periodo fijo entre disparos.
        vTaskDelayUntil(&proximo_despertar, pdMS_TO_TICKS(PERIODO_MEDICION_MS));
    }
}

static void publicar_medicion(rcl_timer_t *temporizador, int64_t ultimo_llamado)
{
    (void)ultimo_llamado;
    if (!temporizador) return;

    static uint32_t ultima_publicada = 0;
    float metros;
    bool valida;
    uint32_t numero;

    taskENTER_CRITICAL(&medicion_mux);
    metros = ultima_distancia_m;
    valida = ultima_valida;
    numero = ultimo_numero;
    taskEXIT_CRITICAL(&medicion_mux);

    if (!valida || numero == 0 || numero == ultima_publicada) return;
    ultima_publicada = numero;

    msg_float.data = metros;
    msg_range.range = metros;

    // Sincroniza reloj con el Agent al iniciar la tarea micro-ROS.
    // Si no hay sincronizacion, se publica stamp=0 (no hora UTC inventada).
    if (rmw_uros_epoch_synchronized()) {
        int64_t ns = rmw_uros_epoch_nanos();
        if (ns > 0) {
            msg_range.header.stamp.sec = (int32_t)(ns / 1000000000LL);
            msg_range.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
        }
    }

    rcl_ret_t a = rcl_publish(&pub_float, &msg_float, NULL);
    rcl_ret_t b = rcl_publish(&pub_range, &msg_range, NULL);
    if (a != RCL_RET_OK || b != RCL_RET_OK) {
        ESP_LOGW(ETIQUETA, "Fallo publicacion Float32=%ld Range=%ld", (long)a, (long)b);
    }
}

static void tarea_micro_ros(void *arg)
{
    (void)arg;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_init_options_t init = rcl_get_zero_initialized_init_options();
    ROS_CHECK(rcl_init_options_init(&init, allocator));

    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init);
    ROS_CHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                               CONFIG_MICRO_ROS_AGENT_PORT,
                                               rmw_options));

    rclc_support_t support;
    ROS_CHECK(rclc_support_init_with_options(&support, 0, NULL, &init, &allocator));

    // No abortar si falla sincronizacion; solo queda timestamp=0.
    if (rmw_uros_sync_session(1000) != RMW_RET_OK) {
        ESP_LOGW(ETIQUETA, "Sincronizacion de tiempo no disponible");
    }

    rcl_node_t nodo = rcl_get_zero_initialized_node();
    ROS_CHECK(rclc_node_init_default(&nodo, "esp32_ultrasonido", "", &support));

    pub_float = rcl_get_zero_initialized_publisher();
    pub_range = rcl_get_zero_initialized_publisher();
    ROS_CHECK(rclc_publisher_init_default(
        &pub_float, &nodo,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/ultrasonic/distance"));
    ROS_CHECK(rclc_publisher_init_default(
        &pub_range, &nodo,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "/ultrasonic/range"));

    msg_float.data = 0.0f;
    if (!sensor_msgs__msg__Range__init(&msg_range)) {
        ESP_LOGE(ETIQUETA, "Sin memoria para mensaje Range");
        vTaskDelete(NULL);
        return;
    }
    if (!rosidl_runtime_c__String__assign(&msg_range.header.frame_id, "ultrasonic_link")) {
        ESP_LOGE(ETIQUETA, "No se pudo fijar frame_id");
        vTaskDelete(NULL);
        return;
    }
    msg_range.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    msg_range.field_of_view = 15.0f * 3.14159265f / 180.0f; // valor nominal del datasheet
    msg_range.min_range = 0.02f;
    msg_range.max_range = 4.0f;
    msg_range.header.stamp.sec = 0;
    msg_range.header.stamp.nanosec = 0;

    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    ROS_CHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(PERIODO_PUBLICACION_MS), publicar_medicion));
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    ROS_CHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    ROS_CHECK(rclc_executor_add_timer(&executor, &timer));

    ESP_LOGI(ETIQUETA, "Publicadores activos: /ultrasonic/distance y /ultrasonic/range");
    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(ultrasonido_init());
    xTaskCreate(tarea_medicion, "rmt_ultrasonido", 4096, NULL, 5, NULL);

    // Wi-Fi y transporte UDP nativos del componente micro-ROS.
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    esp_err_t red = uros_network_interface_initialize();
    if (red != ESP_OK) {
        ESP_LOGE(ETIQUETA, "No se pudo iniciar la red: %s; medicion serial continua", esp_err_to_name(red));
        return;
    }
#endif

    xTaskCreate(tarea_micro_ros, "microros_ultrasonido",
                10240, NULL,
                5, NULL);
}