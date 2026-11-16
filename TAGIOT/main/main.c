/*
 * Firmware principal:
 * - Wi-Fi bloqueante inicializado direto no main.
 * - MPU-6050 monitorando pancadas.
 * - GPS NEO-M8N lendo coordenadas.
 * - Task MQTT enviando os dados em JSON.
 */

#include <stdio.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include "mpu6050_impact.h"
#include "gps_neo_m8n.h"
#include "mqtt_telemetry.h"

#define TAG "MAIN"

// Wi-Fi
#define WIFI_SSID          "AP 404"
#define WIFI_PASS          "0utr4s3nhaB0@"

// Broker MQTT público sem criptografia
#define MQTT_BROKER_URI    "mqtt://broker.emqx.io:1883"
#define MQTT_TOPIC_IMPACT  "testtopic/dados"
#define MQTT_QUEUE_LENGTH  8

// Pinos
#define PINO_ALERTA GPIO_NUM_25
#define PINO_LED    GPIO_NUM_2

// Configuração I2C/MPU
#define MPU_I2C_PORT       I2C_NUM_0
#define MPU_I2C_SDA_GPIO   21
#define MPU_I2C_SCL_GPIO   22
#define MPU_I2C_FREQ_HZ    400000
#define MPU_I2C_ADDRESS    0x68
#define MPU_SAMPLE_PERIOD_MS       5
#define MPU_IMPACT_THRESHOLD_MPS2  35.0f
#define IMPACT_QUEUE_LENGTH        8

// Configuração UART/GPS
#define GPS_UART_PORT      UART_NUM_2
#define GPS_UART_RX_GPIO   16
#define GPS_UART_TX_GPIO   17
#define GPS_BAUD_RATE      9600
#define GPS_RX_BUFFER_SIZE 2048

static Gps_Data_t s_st_gps_data_at_impact;
static bool s_b_gps_data_available_at_impact;
static Mpu6050_ImpactEvent_t s_st_last_impact;
static QueueHandle_t s_h_impact_queue;
static volatile bool s_b_wifi_got_ip = false;

static void prv_MAIN_WifiEventHandler(void *p_v_arg, esp_event_base_t e_event_base,
                                      int32_t s32_event_id, void *p_v_event_data)
{
    (void)p_v_arg;
    (void)p_v_event_data;
    if (e_event_base == WIFI_EVENT && s32_event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (e_event_base == WIFI_EVENT && s32_event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_b_wifi_got_ip = false;
        esp_wifi_connect();
    } else if (e_event_base == IP_EVENT && s32_event_id == IP_EVENT_STA_GOT_IP) {
        s_b_wifi_got_ip = true;
    }
}

static esp_err_t prv_MAIN_InitI2cForMpu(void)
{
    i2c_config_t st_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MPU_I2C_SDA_GPIO,
        .scl_io_num = MPU_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MPU_I2C_FREQ_HZ,
#if ESP_IDF_VERSION_MAJOR >= 5
        .clk_flags = 0,
#endif
    };
    esp_err_t s32_err = i2c_param_config(MPU_I2C_PORT, &st_config);
    if (s32_err != ESP_OK) return s32_err;
    s32_err = i2c_driver_install(MPU_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (s32_err == ESP_ERR_INVALID_STATE) return ESP_OK;
    return s32_err;
}

static void prv_MAIN_GpsReaderTask(void *p_v_parameters)
{
    (void)p_v_parameters;
    for (;;) {
        esp_err_t s32_err = GPS_Update();
        if (s32_err != ESP_OK) {
            ESP_LOGW(TAG, "Falha ao ler GPS: %s", esp_err_to_name(s32_err));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    esp_err_t s32_err;

    gpio_reset_pin(PINO_ALERTA);
    gpio_set_direction(PINO_ALERTA, GPIO_MODE_OUTPUT);
    gpio_set_level(PINO_ALERTA, 0);

    gpio_reset_pin(PINO_LED);
    gpio_set_direction(PINO_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(PINO_LED, 0);

    /* --- Conexão Wi-Fi Serial --- */
    s32_err = nvs_flash_init();
    if (s32_err == ESP_ERR_NVS_NO_FREE_PAGES || s32_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        s32_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(s32_err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t st_wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&st_wifi_init_cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &prv_MAIN_WifiEventHandler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &prv_MAIN_WifiEventHandler, NULL, NULL));

    wifi_config_t st_wifi_cfg = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &st_wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Conectando ao Wi-Fi...");
    while (!s_b_wifi_got_ip) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "Wi-Fi conectado!");

    /* --- Inicialização dos Periféricos --- */
    s_h_impact_queue = xQueueCreate(IMPACT_QUEUE_LENGTH, sizeof(Mpu6050_ImpactEvent_t));
    if (s_h_impact_queue == NULL) return;

    s32_err = prv_MAIN_InitI2cForMpu();
    if (s32_err != ESP_OK) return;

    Mpu6050_ImpactConfig_t st_mpu_config = {
        .h_event_queue = s_h_impact_queue,
        .u32_sample_period_ms = MPU_SAMPLE_PERIOD_MS,
        .f32_impact_accel_threshold_mps2 = MPU_IMPACT_THRESHOLD_MPS2,
        .u8_accel_range = 3,
        .u8_gyro_range = 0
    };
    s32_err = MPU6050_ImpactInit(MPU_I2C_PORT, MPU_I2C_ADDRESS, &st_mpu_config);
    if (s32_err != ESP_OK) return;

    s32_err = GPS_Init(GPS_UART_PORT, GPS_UART_RX_GPIO, GPS_UART_TX_GPIO, GPS_BAUD_RATE, GPS_RX_BUFFER_SIZE);
    if (s32_err != ESP_OK) return;

    Mqtt_TelemetryConfig_t st_mqtt_config = {
        .p_c_broker_uri = MQTT_BROKER_URI,
        .p_c_topic = MQTT_TOPIC_IMPACT,
        .sz_queue_length = MQTT_QUEUE_LENGTH
    };
    s32_err = MQTT_TelemetryInit(&st_mqtt_config);
    if (s32_err != ESP_OK) return;

    /* --- Criação das Tasks --- */
    xTaskCreate(MPU6050_ImpactTask, "mpu6050", 4096, NULL, configMAX_PRIORITIES - 3, NULL);
    xTaskCreate(prv_MAIN_GpsReaderTask, "gps", 3072, NULL, 5, NULL);
    xTaskCreate(MQTT_TelemetryTask, "mqtt", 4096, NULL, 6, NULL);

    ESP_LOGI(TAG, "Sistema rodando...");

    for (;;) {
        if (xQueueReceive(s_h_impact_queue, &s_st_last_impact, portMAX_DELAY) == pdTRUE) {
            s_b_gps_data_available_at_impact = GPS_GetData(&s_st_gps_data_at_impact);

            ESP_LOGW(TAG,"IMPACTO! Acel=%.2f m/s2 | Vel=%.2f m/s | Temp=%.1f C",
			 s_st_last_impact.f32_acceleration_mps2,
			 s_st_last_impact.f32_velocity_mps,
			 s_st_last_impact.f32_temperature_c);

            Mqtt_TelemetryData_t st_telemetry_data = {
                .st_impact = s_st_last_impact,
                .st_gps = s_st_gps_data_at_impact,
                .b_gps_available = s_b_gps_data_available_at_impact
            };

            MQTT_TelemetrySend(&st_telemetry_data, 0);

            for (int i = 0; i < 10; i++) {
                gpio_set_level(PINO_LED, 1);
                gpio_set_level(PINO_ALERTA, 1);
                vTaskDelay(pdMS_TO_TICKS(250));
                gpio_set_level(PINO_ALERTA, 0);
                gpio_set_level(PINO_LED, 0);
                vTaskDelay(pdMS_TO_TICKS(250));
            }
        }
    }
}
