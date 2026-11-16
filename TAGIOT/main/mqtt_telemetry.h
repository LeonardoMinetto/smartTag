#ifndef MQTT_TELEMETRY_H
#define MQTT_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "mpu6050_impact.h"
#include "gps_neo_m8n.h"

#ifdef __cplusplus
extern "C" {
#endif

// Pacote completo com a fotografia do impacto + dados do GPS para envio via MQTT
typedef struct {
    Mpu6050_ImpactEvent_t st_impact;
    Gps_Data_t st_gps;
    bool b_gps_available;
} Mqtt_TelemetryData_t;

// Estrutura de configuração do módulo MQTT
typedef struct {
    const char *p_c_broker_uri;
    const char *p_c_topic;
    size_t sz_queue_length;
} Mqtt_TelemetryConfig_t;

// Inicializa a fila interna e sobe o cliente MQTT da Espressif
esp_err_t MQTT_TelemetryInit(const Mqtt_TelemetryConfig_t *p_st_config);

// Task do FreeRTOS que aguarda dados na fila, formata o JSON e publica no broker
void MQTT_TelemetryTask(void *p_v_parameters);

// Enfileira um pacote de impacto + GPS para ser publicado pela task do MQTT
bool MQTT_TelemetrySend(const Mqtt_TelemetryData_t *p_st_data, TickType_t u32_wait_ticks);

// Retorna se o cliente MQTT está conectado ao broker no momento
bool MQTT_TelemetryIsConnected(void);

#ifdef __cplusplus
}
#endif

#endif /* MQTT_TELEMETRY_H */
