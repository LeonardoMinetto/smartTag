#include "mpu6050_impact.h"
#include <math.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"

#define TAG "MPU6050_IMPACT"

// Mapa dos registradores principais do MPU-6050
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

// Gravidade padrão pra converter de 'g' pra m/s²
#define GRAVITY_MPS2     9.80665f

// Fator de escala (LSB por g) pra cada range do acelerômetro (2g, 4g, 8g, 16g)
static const float s_af32_accel_lsb_per_g[4] = {
    16384.0f, 8192.0f, 4096.0f, 2048.0f
};

// Estado interno do módulo
static int s_s32_i2c_port;
static uint8_t s_u8_address;
static Mpu6050_ImpactConfig_t s_st_cfg;
static float s_f32_vx, s_f32_vy, s_f32_vz; // Acumuladores de velocidade pros 3 eixos

// Escreve 1 byte num registrador do MPU via I2C
static esp_err_t prv_MPU6050_WriteReg(uint8_t u8_reg, uint8_t u8_value)
{
    uint8_t au8_bytes[2] = { u8_reg, u8_value };
    return i2c_master_write_to_device(s_s32_i2c_port, s_u8_address, au8_bytes, 2,
                                      pdMS_TO_TICKS(100));
}

// Lê uma sequência de bytes a partir de um registrador do MPU
static esp_err_t prv_MPU6050_ReadRegs(uint8_t u8_reg, uint8_t *p_u8_data, size_t sz_length)
{
    return i2c_master_write_read_device(s_s32_i2c_port, s_u8_address, &u8_reg, 1,
                                        p_u8_data, sz_length, pdMS_TO_TICKS(100));
}

// Junta o byte alto e o byte baixo que vêm do sensor num inteiro de 16 bits com sinal
static int16_t prv_MPU6050_MakeSignedWord(uint8_t u8_high, uint8_t u8_low)
{
    return (int16_t)(((uint16_t)u8_high << 8) | u8_low);
}

// Lê os 6 bytes do acelerômetro (X, Y, Z) de uma vez só e já converte pra m/s²
static esp_err_t prv_MPU6050_ReadAcceleration(float *p_f32_ax, float *p_f32_ay, float *p_f32_az, float *p_f32_temp)
{
    uint8_t au8_buf[8];

    // Faz burst read começando pelo ACCEL_XOUT_H (0x3B até 0x42)
    esp_err_t s32_err = prv_MPU6050_ReadRegs(REG_ACCEL_XOUT_H, au8_buf, sizeof(au8_buf));
    if (s32_err != ESP_OK) return s32_err;

    // Monta os int16, divide pelo LSB/g da escala escolhida e multiplica pela gravidade
    *p_f32_ax = ((float)prv_MPU6050_MakeSignedWord(au8_buf[0], au8_buf[1]) / s_af32_accel_lsb_per_g[s_st_cfg.u8_accel_range]) * GRAVITY_MPS2;
    *p_f32_ay = ((float)prv_MPU6050_MakeSignedWord(au8_buf[2], au8_buf[3]) / s_af32_accel_lsb_per_g[s_st_cfg.u8_accel_range]) * GRAVITY_MPS2;
    *p_f32_az = ((float)prv_MPU6050_MakeSignedWord(au8_buf[4], au8_buf[5]) / s_af32_accel_lsb_per_g[s_st_cfg.u8_accel_range]) * GRAVITY_MPS2;

    // Formula do proprio datasheet
    *p_f32_temp = ((float)prv_MPU6050_MakeSignedWord(au8_buf[6], au8_buf[7]) / 340.0f) + 36.53f;

    return ESP_OK;
}

// Acorda o MPU-6050 e configura filtros e escalas
esp_err_t MPU6050_ImpactInit(int s32_i2c_port, uint8_t u8_i2c_address,
                             const Mpu6050_ImpactConfig_t *p_st_config)
{
    // Checa se não passaram ponteiro nulo ou configuração fora dos limites aceitáveis
    if (!p_st_config || !p_st_config->h_event_queue ||
        p_st_config->u8_accel_range > 3 || p_st_config->u8_gyro_range > 3 ||
        p_st_config->u32_sample_period_ms == 0 ||
        p_st_config->f32_impact_accel_threshold_mps2 <= 0.0f ||
        (u8_i2c_address != 0x68 && u8_i2c_address != 0x69)) {
        return ESP_ERR_INVALID_ARG;
    }

    // Salva as configurações nas variáveis estáticas do módulo
    s_s32_i2c_port = s32_i2c_port;
    s_u8_address = u8_i2c_address;
    s_st_cfg = *p_st_config;

    // Bate o bit de reset no PWR_MGMT_1 e espera 100ms pro chip reiniciar
    esp_err_t s32_err = prv_MPU6050_WriteReg(REG_PWR_MGMT_1, 0x80);
    if (s32_err != ESP_OK) return s32_err;
    vTaskDelay(pdMS_TO_TICKS(100));

    // Tira do modo sleep e seleciona o PLL do giroscópio X como fonte de clock (mais estável)
    s32_err = prv_MPU6050_WriteReg(REG_PWR_MGMT_1, 0x01);
    if (s32_err != ESP_OK) return s32_err;

    // Configura o filtro passa-baixa digital (DLPF = 3 -> ~44Hz pro accel)
    s32_err = prv_MPU6050_WriteReg(REG_CONFIG, 0x03);
    if (s32_err != ESP_OK) return s32_err;

    // Divisor de taxa de amostragem em 0 pra manter 1 kHz interno
    s32_err = prv_MPU6050_WriteReg(REG_SMPLRT_DIV, 0x00);
    if (s32_err != ESP_OK) return s32_err;

    // Ajusta o fundo de escala do giroscópio (bits 4:3)
    s32_err = prv_MPU6050_WriteReg(REG_GYRO_CONFIG, (uint8_t)(p_st_config->u8_gyro_range << 3));
    if (s32_err != ESP_OK) return s32_err;

    // Ajusta o fundo de escala do acelerômetro (bits 4:3)
    s32_err = prv_MPU6050_WriteReg(REG_ACCEL_CONFIG, (uint8_t)(p_st_config->u8_accel_range << 3));
    if (s32_err != ESP_OK) return s32_err;

    // Lê o registrador WHO_AM_I só pra confirmar se o chip responde o ID padrão (0x68)
    uint8_t u8_who_am_i = 0;
    s32_err = prv_MPU6050_ReadRegs(REG_WHO_AM_I, &u8_who_am_i, 1);
    if (s32_err != ESP_OK) return s32_err;
    if (u8_who_am_i != 0x68) {
        // Clones chineses às vezes respondem 0x70 ou 0x98, então só avisa no log sem travar
        ESP_LOGW(TAG, "WHO_AM_I diferente do padrao: 0x%02X (esperado 0x68)", u8_who_am_i);
    }

    // Zera as velocidades estimadas iniciais
    s_f32_vx = 0.0f;
    s_f32_vy = 0.0f;
    s_f32_vz = 0.0f;
    return ESP_OK;
}

// Loop principal da task que monitora batidas
void MPU6050_ImpactTask(void *p_v_parameters)
{
    (void)p_v_parameters;

    // Guarda o tempo inicial pra calcular o dt de integração
    int64_t s64_previous_us = esp_timer_get_time();

    for (;;) {
        float f32_ax, f32_ay, f32_az, f32_temp;

        // Puxa as leituras atuais de aceleração
        esp_err_t s32_err = prv_MPU6050_ReadAcceleration(&f32_ax, &f32_ay, &f32_az, &f32_temp);
        if (s32_err != ESP_OK) {
            ESP_LOGW(TAG, "Falha na leitura do I2C do acelerometro: %s", esp_err_to_name(s32_err));
            vTaskDelay(pdMS_TO_TICKS(s_st_cfg.u32_sample_period_ms));
            continue;
        }

        // Calcula quanto tempo passou (em segundos) desde a última leitura
        int64_t s64_now_us = esp_timer_get_time();
        float f32_dt = (float)(s64_now_us - s64_previous_us) / 1000000.0f;
        s64_previous_us = s64_now_us;

        // Integra a aceleração no tempo (v = v0 + a*dt) se o dt for coerente
        // Obs: como não estamos descontando o vetor de gravidade por eixo, isso aqui vai dar drift rápido
        if (f32_dt > 0.0f && f32_dt < 0.1f) {
            s_f32_vx += f32_ax * f32_dt;
            s_f32_vy += f32_ay * f32_dt;
            s_f32_vz += f32_az * f32_dt;
        }

        // Calcula o módulo do vetor resultante: sqrt(ax² + ay² + az²)
        float f32_magnitude = sqrtf(f32_ax * f32_ax + f32_ay * f32_ay + f32_az * f32_az);

        // Tira 1g da gravidade pra saber o quanto sobrou de aceleração dinâmica
        float f32_linear_accel_est = fabsf(f32_magnitude - GRAVITY_MPS2);

        // Se passou do limiar configurado, temos um impacto
        if (f32_linear_accel_est >= s_st_cfg.f32_impact_accel_threshold_mps2) {
            Mpu6050_ImpactEvent_t st_event = {
                .b_impact = true,
                .f32_velocity_mps = fabsf(s_f32_vx) + fabsf(s_f32_vy) + fabsf(s_f32_vz),
                .f32_acceleration_mps2 = f32_linear_accel_est,
				.f32_temperature_c = f32_temp,
                .s64_timestamp_us = s64_now_us
            };

            // Tenta mandar pra fila sem bloquear (timeout = 0) pra não atrasar a amostragem
            if (xQueueSend(s_st_cfg.h_event_queue, &st_event, 0) != pdTRUE) {
                ESP_LOGW(TAG, "Fila de impactos cheia, descartando evento");
            }

            // Dá um tempinho de 100ms (debounce) pra mesma pancada não engatilhar vários eventos seguidos
            vTaskDelay(pdMS_TO_TICKS(100));

            // Zera os acumuladores e atualiza o relógio pós-delay
            s_f32_vx = 0.0f;
            s_f32_vy = 0.0f;
            s_f32_vz = 0.0f;
            s64_previous_us = esp_timer_get_time();
            continue;
        }

        // Espera o período configurado antes da próxima amostra
        vTaskDelay(pdMS_TO_TICKS(s_st_cfg.u32_sample_period_ms));
    }
}

// Wrapper simples pro xQueueReceive
bool MPU6050_ImpactReceive(Mpu6050_ImpactEvent_t *p_st_event,
                           TickType_t u32_wait_ticks)
{
    if (!p_st_event || !s_st_cfg.h_event_queue) return false;
    return xQueueReceive(s_st_cfg.h_event_queue, p_st_event, u32_wait_ticks) == pdTRUE;
}
