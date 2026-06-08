#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

// Headers internos do ESP-Drone (Certifique-se de que os paths estão corretos no seu projeto)
#include "crtp.h"

static const char *TAG = "CRTP_SERIAL_LINK";

// ---------------------------------------------------------
// Definições de Hardware e UART (Padrão ESP-IDF)
// ---------------------------------------------------------
#define CRTP_UART_NUM      UART_NUM_1
#define CRTP_TX_PIN        43       
#define CRTP_RX_PIN        44       
#define CRTP_BAUDRATE      115200   
#define UART_BUF_SIZE      1024

// ---------------------------------------------------------
// Definições do Protocolo CRTP Serial Customizado
// ---------------------------------------------------------
#define CRTP_START_BYTE    0xAA     

typedef struct {
    uint8_t header;
    uint8_t size;
    uint8_t data[CRTP_MAX_DATA_SIZE];
    uint8_t checksum;
} CrtpRxPacket_t;

typedef enum { 
    STATE_WAIT_START, 
    STATE_HEADER, 
    STATE_SIZE, 
    STATE_DATA, 
    STATE_CHECKSUM 
} CrtpRxState_t;

static CrtpRxState_t rx_state = STATE_WAIT_START;
static CrtpRxPacket_t rx_packet;
static uint8_t data_idx = 0;
static uint8_t calculated_checksum = 0;

// ---------------------------------------------------------
// Funções de Envio de ACK via UART
// ---------------------------------------------------------
static void send_crtp_ack(uint8_t header) {
    uint8_t ack_packet[5];
    ack_packet[0] = CRTP_START_BYTE; 
    ack_packet[1] = CRTP_START_BYTE;
    ack_packet[2] = header;          
    ack_packet[3] = 0x00;
    ack_packet[4] = header + 0x00;

    uart_write_bytes(CRTP_UART_NUM, (const char*)ack_packet, sizeof(ack_packet));
}

// ---------------------------------------------------------
// Máquina de Estados CRTP e Roteamento Interno do Firmware
// ---------------------------------------------------------
static void process_crtp_byte(uint8_t byte) {
    switch (rx_state) {
        case STATE_WAIT_START:
            if (byte == CRTP_START_BYTE) rx_state = STATE_HEADER;
            break;

        case STATE_HEADER:
            if (byte == CRTP_START_BYTE) break; // Ignora segundos bytes de start duplicados
            rx_packet.header = byte;
            calculated_checksum = byte;
            rx_state = STATE_SIZE;
            break;

        case STATE_SIZE:
            rx_packet.size = byte;
            calculated_checksum += byte;
            if (rx_packet.size > CRTP_MAX_DATA_SIZE) {
                rx_state = STATE_WAIT_START;
            } else if (rx_packet.size == 0) {
                rx_state = STATE_CHECKSUM;
            } else {
                data_idx = 0;
                rx_state = STATE_DATA;
            }
            break;

        case STATE_DATA:
            rx_packet.data[data_idx++] = byte;
            calculated_checksum += byte;
            if (data_idx >= rx_packet.size) rx_state = STATE_CHECKSUM;
            break;

        case STATE_CHECKSUM:
            rx_packet.checksum = byte;
            
            // Validação do pacote serial recebido do simulador
            if (calculated_checksum == rx_packet.checksum) {
                
                // --- ROTEAMENTO CRTP PARA O FIRMWARE ---
                // Criamos a estrutura de pacote oficial da pilha do ESP-Drone
                CRTPPacket internal_packet;
                internal_packet.header = rx_packet.header;
                internal_packet.size = rx_packet.size;
                memcpy(internal_packet.data, rx_packet.data, rx_packet.size);
                
                // Envia o pacote para a fila interna do Crazyflie. 
                // Daqui, o crtp.c despacha automaticamente para o commander correspondente.
                if (crtpRxQueuePost(&internal_packet)) {
                    // Opcional: Log de depuração (recomenda-se comentar em voo para performance)
                    ESP_LOGD(TAG, "Pacote CRTP despachado com sucesso. Porta: %d", (rx_packet.header >> 4) & 0x0F);
                } else {
                    ESP_LOGW(TAG, "Fila de recepção CRTP cheia!");
                }
                
                // Retorna o ACK via Serial para manter estável o handshake do seu Transmissor
                send_crtp_ack(rx_packet.header);
            } else {
                ESP_LOGE(TAG, "Erro: Falha de Checksum no pacote serial!");
            }
            
            rx_state = STATE_WAIT_START;
            break;
    }
}

// ---------------------------------------------------------
// Task Principal do FreeRTOS para Escuta da UART
// ---------------------------------------------------------
static void crtp_serial_link_task(void *pvParameters) {
    uint8_t *data = (uint8_t *) malloc(UART_BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE(TAG, "Falha ao alocar buffer da UART");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Link Serial CRTP iniciado. Aguardando pacotes...");

    while (1) {
        // Leitura contínua bloqueante por timeout (não consome CPU enquanto ociosa)
        int len = uart_read_bytes(CRTP_UART_NUM, data, UART_BUF_SIZE, 20 / portTICK_PERIOD_MS);
        for (int i = 0; i < len; i++) {
            process_crtp_byte(data[i]);
        }
    }
}

// ---------------------------------------------------------
// Função de Inicialização do Módulo (Chamada no startup do drone)
// ---------------------------------------------------------
void crtpSerialLinkInit(void) {
    uart_config_t uart_config = {
        .baud_rate = CRTP_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Configura os parâmetros e instala o driver da UART
    ESP_ERROR_CHECK(uart_param_config(CRTP_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(CRTP_UART_NUM, CRTP_TX_PIN, CRTP_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(CRTP_UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));

    // Cria a task responsável por processar os bytes recebidos
    xTaskCreatePinnedToCore(crtp_serial_link_task, "crtpSerialLink", 4096, NULL, 5, NULL, 1);
}