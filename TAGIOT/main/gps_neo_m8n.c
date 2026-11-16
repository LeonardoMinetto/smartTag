#include "gps_neo_m8n.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Tamanhos máximos pra linha NMEA e pra quantidade de campos separados por vírgula
#define GPS_LINE_MAX 128
#define GPS_FIELD_MAX 20

// Variáveis estáticas pra manter o estado interno do driver
static uart_port_t s_e_uart_num = UART_NUM_MAX; // Guarda qual porta UART estamos usando
static bool s_b_initialized;                    // Flag pra evitar inicializar duas vezes ou ler sem iniciar
static char s_ac_line[GPS_LINE_MAX];            // Buffer onde vamos montando a frase byte a byte
static size_t s_sz_line_len;                    // Posição atual de escrita no buffer da linha
static Gps_Data_t s_st_data;                    // Cache com os últimos dados decodificados do GPS
static bool s_b_have_data;                      // Indica se já recebemos algum pacote desde que ligou
static SemaphoreHandle_t s_h_data_mutex;        // Mutex pra task do GPS e o main não atropelarem a struct

// Valida o checksum da frase NMEA (faz o XOR de tudo entre o '$' e o '*')
static bool prv_GPS_CheckNmeaChecksum(const char *p_c_line)
{
    // Toda frase NMEA tem que começar com '$'
    if (p_c_line == NULL || p_c_line[0] != '$') return false;

    // Procura o '*' onde começa o checksum e garante que tem os 2 dígitos hex depois dele
    const char *p_c_star = strchr(p_c_line, '*');
    if (p_c_star == NULL || p_c_star[1] == '\0' || p_c_star[2] == '\0') return false;

    // Pega os 2 caracteres depois do '*' e converte de texto hexa pra inteiro
    unsigned int u32_expected = 0;
    char ac_hex[3] = { p_c_star[1], p_c_star[2], '\0' };
    char *p_c_end = NULL;
    u32_expected = (unsigned int)strtoul(ac_hex, &p_c_end, 16);

    // Se não conseguiu converter os dois caracteres certinho, descarta
    if (p_c_end == ac_hex || *p_c_end != '\0') return false;

    // Calcula o XOR de todos os bytes entre o '$' e o '*'
    unsigned int u32_actual = 0;
    for (const char *p_c_curr = p_c_line + 1; p_c_curr < p_c_star; ++p_c_curr) {
        u32_actual ^= (unsigned char)*p_c_curr;
    }

    // Bate o valor calculado com o que veio na mensagem
    return u32_actual == u32_expected;
}

// Converte o formato bizarro do NMEA (ddmm.mmmm) pra graus decimais normais
static double prv_GPS_ConvertNmeaCoordToDeg(const char *p_c_value, const char *p_c_hemisphere)
{
    // Se o campo veio vazio (comum quando o GPS ainda não tem fix), retorna zero
    if (p_c_value == NULL || p_c_hemisphere == NULL || p_c_value[0] == '\0') return 0.0;

    // Converte a string bruta pra double
    double f64_raw = strtod(p_c_value, NULL);

    // Isola a parte inteira dos graus dividindo por 100
    int s32_degrees = (int)(f64_raw / 100.0);

    // O que sobrar são os minutos
    double f64_minutes = f64_raw - ((double)s32_degrees * 100.0);

    // Junta os graus com os minutos divididos por 60
    double f64_result = (double)s32_degrees + f64_minutes / 60.0;

    // Se for Sul ou Oeste, inverte o sinal
    if (p_c_hemisphere[0] == 'S' || p_c_hemisphere[0] == 'W') {
        f64_result = -f64_result;
    }
    return f64_result;
}

// Quebra a linha NMEA em vários ponteiros onde tem vírgula (modifica a string original)
static int prv_GPS_SplitFields(char *p_c_sentence, char *ap_c_fields[], int s32_max_fields)
{
    int s32_count = 0;
    char *p_c_curr = p_c_sentence;

    // Corta o checksum fora colocando um '\0' em cima do '*'
    char *p_c_star = strchr(p_c_curr, '*');
    if (p_c_star != NULL) *p_c_star = '\0';

    // Vai trocando cada vírgula por '\0' e salvando o início de cada pedaço no array
    while (s32_count < s32_max_fields) {
        ap_c_fields[s32_count++] = p_c_curr;
        char *p_c_comma = strchr(p_c_curr, ',');
        if (p_c_comma == NULL) break; // Acabaram as vírgulas
        *p_c_comma = '\0';
        p_c_curr = p_c_comma + 1;
    }

    return s32_count;
}

// Analisa uma linha NMEA completa e atualiza a struct estática se for RMC ou GGA
static void prv_GPS_ParseNmeaLine(char *p_c_line)
{
    // Primeiro vê se a mensagem não corrompeu no caminho
    if (!prv_GPS_CheckNmeaChecksum(p_c_line)) return;

    // Cria uma cópia local porque a função de split vai picotar a string com '\0'
    char ac_work[GPS_LINE_MAX];
    size_t sz_len = strlen(p_c_line);
    if (sz_len >= sizeof(ac_work)) return;
    memcpy(ac_work, p_c_line, sz_len + 1);

    // Separa os campos por vírgula
    char *ap_c_fields[GPS_FIELD_MAX] = {0};
    int s32_count = prv_GPS_SplitFields(ac_work, ap_c_fields, GPS_FIELD_MAX);

    // O primeiro campo tem que ter pelo menos 6 caracteres (ex: "$GNRMC")
    if (s32_count == 0 || strlen(ap_c_fields[0]) < 6) return;

    // Pega só as últimas 3 letras do identificador pra funcionar tanto com $GPRMC quanto $GNRMC, etc.
    const char *p_c_kind = ap_c_fields[0] + strlen(ap_c_fields[0]) - 3;

    if (strcmp(p_c_kind, "RMC") == 0) {
        // Pacote RMC: $--RMC,hora,status,lat,N/S,lon,E/W,vel_nos,curso,...
        if (s32_count < 9) return;

        // No RMC, o campo 2 diz se o dado é válido ('A' = ativo, 'V' = void/inválido)
        if (ap_c_fields[2][0] != 'A') {
            s_st_data.b_position_valid = false;
            s_st_data.b_has_rmc = true;
            s_b_have_data = true;
            return;
        }

        // Converte coordenadas, passa velocidade de nós pra km/h (* 1.852) e pega o curso
        s_st_data.f64_latitude_deg = prv_GPS_ConvertNmeaCoordToDeg(ap_c_fields[3], ap_c_fields[4]);
        s_st_data.f64_longitude_deg = prv_GPS_ConvertNmeaCoordToDeg(ap_c_fields[5], ap_c_fields[6]);
        s_st_data.f32_speed_kmh = (float)(strtod(ap_c_fields[7], NULL) * 1.852);
        s_st_data.f32_course_deg = (float)strtod(ap_c_fields[8], NULL);
        s_st_data.b_position_valid = true;
        s_st_data.b_has_rmc = true;
        s_b_have_data = true;
    } else if (strcmp(p_c_kind, "GGA") == 0) {
        // Pacote GGA: $--GGA,hora,lat,N/S,lon,E/W,qualidade,satelites,HDOP,altitude,M,...
        if (s32_count < 10) return;

        // No GGA, o campo 6 é o indicador de qualidade do fix (0 = sem fix)
        int s32_quality = atoi(ap_c_fields[6]);
        if (s32_quality == 0) {
            s_st_data.b_position_valid = false;
            s_st_data.b_has_gga = true;
            s_b_have_data = true;
            return;
        }

        // Pega coordenadas, número de satélites e altitude
        s_st_data.f64_latitude_deg = prv_GPS_ConvertNmeaCoordToDeg(ap_c_fields[2], ap_c_fields[3]);
        s_st_data.f64_longitude_deg = prv_GPS_ConvertNmeaCoordToDeg(ap_c_fields[4], ap_c_fields[5]);
        s_st_data.u8_satellites = (uint8_t)atoi(ap_c_fields[7]);
        s_st_data.f32_altitude_m = (float)strtod(ap_c_fields[9], NULL);
        s_st_data.b_position_valid = true;
        s_st_data.b_has_gga = true;
        s_b_have_data = true;
    }
}

// Inicializa a porta UART pro módulo GPS
esp_err_t GPS_Init(uart_port_t e_uart_num, int s32_rx_gpio, int s32_tx_gpio,
                   int s32_baud_rate, size_t sz_rx_buffer_size)
{
    // Evita reinicializar por engano e valida os parâmetros básicos
    if (s_b_initialized) return ESP_ERR_INVALID_STATE;
    if (e_uart_num < UART_NUM_0 || e_uart_num >= UART_NUM_MAX ||
        s32_rx_gpio < 0 || s32_tx_gpio < 0 || s32_baud_rate <= 0 || sz_rx_buffer_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Configuração padrão 8N1 sem controle de fluxo
    uart_config_t st_config = {
        .baud_rate = s32_baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION_MAJOR >= 5
        .source_clk = UART_SCLK_DEFAULT,
#else
        .source_clk = UART_SCLK_APB,
#endif
    };

    // Instala o driver da UART alocando só o buffer de RX
    esp_err_t s32_err = uart_driver_install(e_uart_num, sz_rx_buffer_size, 0, 0, NULL, 0);
    if (s32_err != ESP_OK) return s32_err;

    // Aplica os parâmetros de baudrate, paridade, etc.
    s32_err = uart_param_config(e_uart_num, &st_config);
    if (s32_err != ESP_OK) {
        uart_driver_delete(e_uart_num);
        return s32_err;
    }

    // Amarra os pinos físicos de TX e RX na UART escolhida
    s32_err = uart_set_pin(e_uart_num, s32_tx_gpio, s32_rx_gpio,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (s32_err != ESP_OK) {
        uart_driver_delete(e_uart_num);
        return s32_err;
    }

    // Cria o mutex se ainda não existir
    if (s_h_data_mutex == NULL) {
        s_h_data_mutex = xSemaphoreCreateMutex();
        if (s_h_data_mutex == NULL) {
            uart_driver_delete(e_uart_num);
            return ESP_ERR_NO_MEM;
        }
    }

    // Zera os buffers e marca como inicializado
    s_e_uart_num = e_uart_num;
    s_b_initialized = true;
    s_sz_line_len = 0;
    s_b_have_data = false;
    memset(&s_st_data, 0, sizeof(s_st_data));
    memset(s_ac_line, 0, sizeof(s_ac_line));
    return ESP_OK;
}

// Consome os bytes recebidos na UART e monta as frases NMEA
esp_err_t GPS_Update(void)
{
    if (!s_b_initialized) return ESP_ERR_INVALID_STATE;

    uint8_t u8_byte;
    for (;;) {
        // Lê 1 byte com timeout zero; se não tiver nada no buffer da UART, sai do loop na hora
        int s32_bytes_read = uart_read_bytes(s_e_uart_num, &u8_byte, 1, 0);
        if (s32_bytes_read <= 0) break;

        char c_char = (char)u8_byte;

        // Ignora o '\r' do final de linha padrão (\r\n)
        if (c_char == '\r') continue;

        // Quando bate o '\n', fechou uma linha completa
        if (c_char == '\n') {
            s_ac_line[s_sz_line_len] = '\0';
            if (s_sz_line_len > 0) {
                // Trava o mutex, faz o parse da linha e solta o mutex
                if (s_h_data_mutex != NULL) xSemaphoreTake(s_h_data_mutex, portMAX_DELAY);
                prv_GPS_ParseNmeaLine(s_ac_line);
                if (s_h_data_mutex != NULL) xSemaphoreGive(s_h_data_mutex);
            }
            // Reseta o índice pra começar a montar a próxima linha
            s_sz_line_len = 0;
        } else if (s_sz_line_len < sizeof(s_ac_line) - 1) {
            // Vai empilhando o caractere no buffer da linha
            s_ac_line[s_sz_line_len++] = c_char;
        } else {
            // Se entrou lixo e a linha passou de 127 bytes sem dar '\n', descarta e zera
            s_sz_line_len = 0;
        }
    }

    return ESP_OK;
}

// Entrega uma cópia segura dos dados atuais do GPS
bool GPS_GetData(Gps_Data_t *p_st_out)
{
    if (p_st_out == NULL || !s_b_initialized || s_h_data_mutex == NULL) return false;

    // Pega o mutex pra garantir que a task do GPS não está alterando a struct no meio da cópia
    xSemaphoreTake(s_h_data_mutex, portMAX_DELAY);
    bool b_available = s_b_have_data;
    if (b_available) *p_st_out = s_st_data;
    xSemaphoreGive(s_h_data_mutex);

    return b_available;
}

// Só retorna a flag de inicialização
bool GPS_IsInitialized(void)
{
    return s_b_initialized;
}

// Desliga o driver da UART e limpa o estado
esp_err_t GPS_Deinit(void)
{
    if (!s_b_initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t s32_err = uart_driver_delete(s_e_uart_num);
    if (s32_err == ESP_OK) {
        s_b_initialized = false;
        s_e_uart_num = UART_NUM_MAX;
        s_sz_line_len = 0;
    }
    return s32_err;
}
