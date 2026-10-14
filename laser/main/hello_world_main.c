// Lectura VL53L0X + micro-ROS sobre el mismo ESP32/Agent del TP ultrasonico.
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "laser.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/range.h>
#include <rosidl_runtime_c/string_functions.h>

#define TAG "LASER"
#define MEDIR_CADA_MS 100      // frecuencia objetivo: 10 Hz (con mediciones exitosas)
#define REVISAR_ROS_MS 20     // no fija la frecuencia de medicion

static portMUX_TYPE datos_lock = portMUX_INITIALIZER_UNLOCKED;
static float ultima_distancia_m = 0.0f;
static bool ultima_valida = false;
static uint32_t numero_lectura = 0;

static rcl_publisher_t publicador_float;
static rcl_publisher_t publicador_range;
static std_msgs__msg__Float32 mensaje_float;
static sensor_msgs__msg__Range mensaje_range;

#define ROS_CHECK(x) do { \
    rcl_ret_t rc = (x); \
    if (rc != RCL_RET_OK) { \
        ESP_LOGE(TAG, "Error ROS %ld en linea %d", (long)rc, __LINE__); \
        vTaskDelete(NULL); \
        return; \
    } \
} while (0)

static void tarea_leer_laser(void *argumento)
{
    (void)argumento;
    TickType_t proximo_ciclo = xTaskGetTickCount();
    for (;;) {
        float metros = 0;
        uint16_t mm = 0;
        uint8_t estado_sensor = 255;
        esp_err_t resultado = laser_medicion(&metros, &mm, &estado_sensor);

        taskENTER_CRITICAL(&datos_lock);
        ultima_valida = (resultado == ESP_OK);
        if (resultado == ESP_OK) ultima_distancia_m = metros;
        numero_lectura++;
        taskEXIT_CRITICAL(&datos_lock);

        if (resultado == ESP_OK) {
            ESP_LOGI(TAG, "Distancia: %u mm = %.3f m", (unsigned)mm, metros);
        } else {
            ESP_LOGW(TAG, "Medicion invalida: %s, estado ST=%u",
                     esp_err_to_name(resultado), (unsigned)estado_sensor);
        }
        vTaskDelayUntil(&proximo_ciclo, pdMS_TO_TICKS(MEDIR_CADA_MS));
    }
}

static void publicar_si_hay_nueva_medicion(rcl_timer_t *temporizador, int64_t ultima_llamada)
{
    (void)ultima_llamada;
    if (!temporizador) return;
    static uint32_t ultimo_numero_publicado = 0;
    float distancia;
    bool valida;
    uint32_t numero;

    taskENTER_CRITICAL(&datos_lock);
    distancia = ultima_distancia_m;
    valida = ultima_valida;
    numero = numero_lectura;
    taskEXIT_CRITICAL(&datos_lock);

    if (!valida || numero == 0 || numero == ultimo_numero_publicado) return;
    ultimo_numero_publicado = numero;
    mensaje_float.data = distancia;
    mensaje_range.range = distancia;

    if (rmw_uros_epoch_synchronized()) {
        int64_t ns = rmw_uros_epoch_nanos();
        if (ns > 0) {
            mensaje_range.header.stamp.sec = (int32_t)(ns / 1000000000LL);
            mensaje_range.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
        }
    }

    rcl_ret_t a = rcl_publish(&publicador_float, &mensaje_float, NULL);
    rcl_ret_t b = rcl_publish(&publicador_range, &mensaje_range, NULL);
    if (a != RCL_RET_OK || b != RCL_RET_OK)
        ESP_LOGW(TAG, "Error al publicar Float32=%ld Range=%ld", (long)a, (long)b);
}

static void tarea_ros(void *argumento)
{
    (void)argumento;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_init_options_t opciones = rcl_get_zero_initialized_init_options();
    ROS_CHECK(rcl_init_options_init(&opciones, allocator));
    rmw_init_options_t *opciones_rmw = rcl_init_options_get_rmw_init_options(&opciones);
    ROS_CHECK(rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, opciones_rmw));

    rclc_support_t support;
    ROS_CHECK(rclc_support_init_with_options(&support, 0, NULL, &opciones, &allocator));
    if (rmw_uros_sync_session(1000) != RMW_RET_OK)
        ESP_LOGW(TAG, "Sincronizacion temporal no disponible");

    rcl_node_t nodo = rcl_get_zero_initialized_node();
    ROS_CHECK(rclc_node_init_default(&nodo, "esp32_laser", "", &support));

    publicador_float = rcl_get_zero_initialized_publisher();
    publicador_range = rcl_get_zero_initialized_publisher();
    ROS_CHECK(rclc_publisher_init_default(&publicador_float, &nodo,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/laser/distance"));
    ROS_CHECK(rclc_publisher_init_default(&publicador_range, &nodo,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "/laser/range"));

    mensaje_float.data = 0.0f;
    if (!sensor_msgs__msg__Range__init(&mensaje_range) ||
        !rosidl_runtime_c__String__assign(&mensaje_range.header.frame_id, "laser_link")) {
        ESP_LOGE(TAG, "No se pudo inicializar mensaje Range");
        vTaskDelete(NULL);
        return;
    }
    mensaje_range.radiation_type = sensor_msgs__msg__Range__INFRARED; // categoria ROS, aunque use laser IR
    mensaje_range.field_of_view = 25.0f * 3.14159265f / 180.0f;
    mensaje_range.min_range = 0.03f; // limite de uso propuesto; verificar con modulo real
    mensaje_range.max_range = 2.0f;  // alcance nominal en condiciones favorables
    mensaje_range.header.stamp.sec = 0;
    mensaje_range.header.stamp.nanosec = 0;

    rcl_timer_t temporizador = rcl_get_zero_initialized_timer();
    ROS_CHECK(rclc_timer_init_default(
        &temporizador, &support, RCL_MS_TO_NS(REVISAR_ROS_MS), publicar_si_hay_nueva_medicion));
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    ROS_CHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    ROS_CHECK(rclc_executor_add_timer(&executor, &temporizador));

    ESP_LOGI(TAG, "Publicando en /laser/distance y /laser/range");
    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(laser_init());
    xTaskCreate(tarea_leer_laser, "vl53l0x_i2c", 4096, NULL, 5, NULL);

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    esp_err_t red = uros_network_interface_initialize();
    if (red != ESP_OK) {
        ESP_LOGE(TAG, "Error Wi-Fi: %s; la medicion local continua", esp_err_to_name(red));
        return;
    }
#endif
    xTaskCreate(tarea_ros, "microros_laser", 10240, NULL, 5, NULL);
}
