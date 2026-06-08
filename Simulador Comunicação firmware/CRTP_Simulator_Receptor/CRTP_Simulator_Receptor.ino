#include <Arduino.h>

// ---------------------------------------------------------
// 1. Definições de Hardware e UART 
// ---------------------------------------------------------
#define CRTP_TX_PIN        43       
#define CRTP_RX_PIN        44       
#define CRTP_BAUDRATE      115200   

// ---------------------------------------------------------
// 2. Definições do Protocolo CRTP Serial
// ---------------------------------------------------------
#define CRTP_START_BYTE    0xAA     
#define CRTP_MAX_DATA_SIZE 30       

typedef struct {
    uint8_t header;
    uint8_t size;
    uint8_t data[CRTP_MAX_DATA_SIZE];
    uint8_t checksum;
} CrtpPacket_t;

typedef enum { 
    STATE_WAIT_START, 
    STATE_HEADER, 
    STATE_SIZE, 
    STATE_DATA, 
    STATE_CHECKSUM 
} CrtpRxState_t;

CrtpRxState_t rx_state = STATE_WAIT_START;
CrtpPacket_t rx_packet;
uint8_t data_idx = 0;
uint8_t calculated_checksum = 0;

// ---------------------------------------------------------
// 3. Funções Auxiliares
// ---------------------------------------------------------
void printHexBuffer(uint8_t* buffer, uint8_t size) {
    Serial.print("DADOS (Hex): ");
    for (int i = 0; i < size; i++) {
        Serial.printf("%02X ", buffer[i]);
    }
    Serial.println(); 
}

void send_crtp_ack(uint8_t header) {
    uint8_t ack_packet[5];
    ack_packet[0] = CRTP_START_BYTE; 
    ack_packet[1] = CRTP_START_BYTE;
    ack_packet[2] = header;          
    ack_packet[3] = 0x00;
    ack_packet[4] = header + 0x00;

    Serial1.write(ack_packet, sizeof(ack_packet));
    Serial.printf("<= ACK Enviado (Confirmacao da porta 0x%02X)\n\n", header);
}

// ---------------------------------------------------------
// 4. Máquina de Estados CRTP e Output em Tela
// ---------------------------------------------------------
void process_crtp_byte(uint8_t byte) {
    switch (rx_state) {
        case STATE_WAIT_START:
            if (byte == CRTP_START_BYTE) rx_state = STATE_HEADER;
            break;

        case STATE_HEADER:
            if (byte == CRTP_START_BYTE) break;
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
            
            // Validação do pacote serial recebido
            if (calculated_checksum == rx_packet.checksum) {
                uint8_t port = (rx_packet.header >> 4) & 0x0F;
                uint8_t channel = rx_packet.header & 0x0F;
                
                Serial.printf("=> RECEBIDO SERIAL | Porta: %d, Canal: %d, Tamanho: %d bytes\n", port, channel, rx_packet.size);
                if(rx_packet.size > 0) {
                    printHexBuffer(rx_packet.data, rx_packet.size);
                }
                
                // Retorna ACK via Serial para manter o handshake do transmissor
                send_crtp_ack(rx_packet.header);
            } else {
                Serial.println("Erro: Falha de Checksum no pacote serial!");
            }
            
            rx_state = STATE_WAIT_START;
            break;
    }
}

// ---------------------------------------------------------
// SETUP
// ---------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== INICIANDO RECEPTOR CRTP LOCAL ===");

    Serial1.begin(CRTP_BAUDRATE, SERIAL_8N1, CRTP_RX_PIN, CRTP_TX_PIN);
    Serial.println("[UART] Aguardando pacotes do transmissor...");
}

// ---------------------------------------------------------
// LOOP
// ---------------------------------------------------------
void loop() {
    while (Serial1.available() > 0) {
        uint8_t incoming_byte = Serial1.read();
        process_crtp_byte(incoming_byte);
    }
}