#include "mqtt_telemetry.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_event.h"
#include "mqtt_client.h"

#define TAG "MQTT_TELEMETRY"

#define MQTT_JSON_BUFFER_MAX 384

static QueueHandle_t s_h_telemetry_queue = NULL;
static esp_mqtt_client_handle_t s_h_mqtt_client = NULL;
static char s_ac_topic[128];
static volatile bool s_b_connected = false;
static bool s_b_initialized = false;

static void prv_MQTT_EventHandler(void *p_v_handler_args, esp_event_base_t e_base,
                                  int32_t s32_event_id, void *p_v_event_data)
{
    (void)p_v_handler_args;
    (void)e_base;
    (void)p_v_event_data;

    switch ((esp_mqtt_event_id_t)s32_event_id) {
        case MQTT_EVENT_CONNECTED:
            s_b_connected = true;
            ESP_LOGI(TAG, "Conectado ao broker MQTT");
            break;
        case MQTT_EVENT_DISCONNECTED:
            s_b_connected = false;
            ESP_LOGW(TAG, "Desconectado do broker MQTT");
            break;
        default:
            break;
    }
}

static bool prv_MQTT_FormatJson(const Mqtt_TelemetryData_t *p_st_data,
                                char *p_c_out_buf, size_t sz_buf_size)
{
    if (p_st_data == NULL || p_c_out_buf == NULL || sz_buf_size == 0) {
        return false;
    }

    int s32_written = snprintf(
        p_c_out_buf, sz_buf_size,
        "{"
        "\"impact\":true,"
        "\"timestamp_us\":%lld,"
        "\"acceleration_mps2\":%.2f,"
        "\"velocity_mps\":%.2f,"
		"\"temperature_c\":%.2f,"
        "\"gps_available\":%s,"
        "\"position_valid\":%s,"
        "\"latitude_deg\":%.7f,"
        "\"longitude_deg\":%.7f,"
        "\"altitude_m\":%.2f,"
        "\"speed_kmh\":%.2f,"
        "\"course_deg\":%.2f,"
        "\"satellites\":%u"
        "}",
        (long long)p_st_data->st_impact.s64_timestamp_us,
        p_st_data->st_impact.f32_acceleration_mps2,
        p_st_data->st_impact.f32_velocity_mps,
		p_st_data->st_impact.f32_temperature_c,
        p_st_data->b_gps_available ? "true" : "false",
        (p_st_data->b_gps_available && p_st_data->st_gps.b_position_valid) ? "true" : "false",
        p_st_data->b_gps_available ? p_st_data->st_gps.f64_latitude_deg : 0.0,
        p_st_data->b_gps_available ? p_st_data->st_gps.f64_longitude_deg : 0.0,
        p_st_data->b_gps_available ? p_st_data->st_gps.f32_altitude_m : 0.0f,
        p_st_data->b_gps_available ? p_st_data->st_gps.f32_speed_kmh : 0.0f,
        p_st_data->b_gps_available ? p_st_data->st_gps.f32_course_deg : 0.0f,
        p_st_data->b_gps_available ? (unsigned)p_st_data->st_gps.u8_satellites : 0U
    );

    return (s32_written > 0 && (size_t)s32_written < sz_buf_size);
}

esp_err_t MQTT_TelemetryInit(const Mqtt_TelemetryConfig_t *p_st_config)
{
    if (s_b_initialized) return ESP_ERR_INVALID_STATE;
    if (p_st_config == NULL || p_st_config->p_c_broker_uri == NULL ||
        p_st_config->p_c_topic == NULL || p_st_config->sz_queue_length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t sz_topic_len = strlen(p_st_config->p_c_topic);
    if (sz_topic_len == 0 || sz_topic_len >= sizeof(s_ac_topic)) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(s_ac_topic, p_st_config->p_c_topic, sz_topic_len + 1);

    s_h_telemetry_queue = xQueueCreate(p_st_config->sz_queue_length, sizeof(Mqtt_TelemetryData_t));
    if (s_h_telemetry_queue == NULL) return ESP_ERR_NO_MEM;

    // Configuração MQTT padrão, sem TLS/criptografia
    esp_mqtt_client_config_t st_mqtt_cfg = {
        .broker.address.uri = p_st_config->p_c_broker_uri,
    };

    s_h_mqtt_client = esp_mqtt_client_init(&st_mqtt_cfg);
    if (s_h_mqtt_client == NULL) {
        vQueueDelete(s_h_telemetry_queue);
        return ESP_FAIL;
    }

    esp_mqtt_client_register_event(s_h_mqtt_client, ESP_EVENT_ANY_ID, prv_MQTT_EventHandler, NULL);
    esp_mqtt_client_start(s_h_mqtt_client);

    s_b_initialized = true;
    return ESP_OK;
}

void MQTT_TelemetryTask(void *p_v_parameters)
{
    (void)p_v_parameters;
    Mqtt_TelemetryData_t st_data;
    char ac_json[MQTT_JSON_BUFFER_MAX];

    for (;;) {
        if (xQueueReceive(s_h_telemetry_queue, &st_data, portMAX_DELAY) == pdTRUE) {
            if (!prv_MQTT_FormatJson(&st_data, ac_json, sizeof(ac_json))) {
                ESP_LOGE(TAG, "Falha ao formatar pacote JSON");
                continue;
            }

            if (s_b_connected && s_h_mqtt_client != NULL) {
                // QoS 1 garante a confirmação de entrega pelo broker
                int s32_msg_id = esp_mqtt_client_publish(s_h_mqtt_client, s_ac_topic, ac_json, 0, 1, 0);
                if (s32_msg_id >= 0) {
                    ESP_LOGI(TAG, "JSON publicado (msg_id=%d): %s", s32_msg_id, ac_json);
                } else {
                    ESP_LOGE(TAG, "Falha na publicacao MQTT");
                }
            } else {
                ESP_LOGW(TAG, "MQTT desconectado, descartando: %s", ac_json);
            }
        }
    }
}

bool MQTT_TelemetrySend(const Mqtt_TelemetryData_t *p_st_data, TickType_t u32_wait_ticks)
{
    if (!s_b_initialized || p_st_data == NULL || s_h_telemetry_queue == NULL) return false;
    return xQueueSend(s_h_telemetry_queue, p_st_data, u32_wait_ticks) == pdTRUE;
}

bool MQTT_TelemetryIsConnected(void)
{
    return s_b_connected;
}
