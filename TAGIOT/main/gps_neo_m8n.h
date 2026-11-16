#ifndef GPS_NEO_M8N_H
#define GPS_NEO_M8N_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

// Estrutura pra guardar os dados do GPS já limpos e convertidos das strings NMEA
typedef struct {
    double f64_latitude_deg;    // Latitude em graus decimais (Norte é +, Sul é -)
    double f64_longitude_deg;   // Longitude em graus decimais (Leste é +, Oeste é -)
    float f32_altitude_m;       // Altitude em metros (vem do pacote GGA)
    float f32_speed_kmh;        // Velocidade convertida pra km/h (vem do pacote RMC)
    float f32_course_deg;       // Direção/curso em graus em relação ao solo (vem do RMC)
    uint8_t u8_satellites;      // Quantidade de satélites travados no fix (vem do GGA)
    bool b_position_valid;      // True se o GPS já pegou fix válido, false se ainda está perdido
    bool b_has_rmc;             // Flag pra saber se já entrou pelo menos uma frase RMC válida
    bool b_has_gga;             // Flag pra saber se já entrou pelo menos uma frase GGA válida
} Gps_Data_t;

// Sobe o driver da UART pros pinos do GPS e cria o mutex de proteção dos dados
esp_err_t GPS_Init(uart_port_t e_uart_num, int s32_rx_gpio, int s32_tx_gpio,
                   int s32_baud_rate, size_t sz_rx_buffer_size);

// Lê o que tiver no buffer da UART sem travar a task e manda pro parser quando fecha uma linha
esp_err_t GPS_Update(void);

// Copia os dados atuais pra fora usando o mutex pra não dar condição de corrida
bool GPS_GetData(Gps_Data_t *p_st_out);

// Retorna se o módulo já foi inicializado ou não
bool GPS_IsInitialized(void);

// Desinstala o driver da UART e reseta o estado do módulo
esp_err_t GPS_Deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* GPS_NEO_M8N_H */
