#ifndef MPU6050_IMPACT_H
#define MPU6050_IMPACT_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// Pacote que é jogado na fila quando o acelerômetro detecta uma pancada
typedef struct {
    bool b_impact;                  // Sempre true quando o evento é gerado
    float f32_velocity_mps;         // Soma simples das velocidades integradas (|Vx| + |Vy| + |Vz|)
    float f32_acceleration_mps2;    // Aceleração linear estimada (descontando 1g da gravidade)
    float f32_temperature_c;        // Temperatura no momento do impacto
    int64_t s64_timestamp_us;       // Tempo em microssegundos desde que o ESP ligou
} Mpu6050_ImpactEvent_t;

// Estrutura pra configurar o comportamento do detector na inicialização
typedef struct {
    QueueHandle_t h_event_queue;             // Fila do FreeRTOS onde os impactos vão cair
    uint32_t u32_sample_period_ms;           // Intervalo entre leituras em ms
    float f32_impact_accel_threshold_mps2;   // Limiar em m/s² acima de 1g pra considerar batida
    uint8_t u8_accel_range;                  // Escala do acelerômetro: 0=2g, 1=4g, 2=8g, 3=16g
    uint8_t u8_gyro_range;                   // Escala do giroscópio: 0=250, 1=500, 2=1000, 3=2000 °/s
} Mpu6050_ImpactConfig_t;

// Configura os registradores do MPU-6050 via I2C (o barramento I2C já deve estar aberto)
esp_err_t MPU6050_ImpactInit(int s32_i2c_port, uint8_t u8_i2c_address,
                             const Mpu6050_ImpactConfig_t *p_st_config);

// Task que fica lendo o sensor em loop e dispara eventos na fila
void MPU6050_ImpactTask(void *p_v_parameters);

// Função auxiliar caso queira puxar um evento da fila diretamente pelo módulo
bool MPU6050_ImpactReceive(Mpu6050_ImpactEvent_t *p_st_event,
                           TickType_t u32_wait_ticks);

#ifdef __cplusplus
}
#endif

#endif
