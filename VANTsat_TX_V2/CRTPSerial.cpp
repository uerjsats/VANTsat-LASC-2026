#include "CRTPSerial.h"
// ---------------------------------------------------------
// Definições de Hardware e Constantes Internas
// ---------------------------------------------------------
#define TX_PIN           43       
#define RX_PIN           44      
#define BAUDRATE         115200   
#define CRTP_START_BYTE  0xAA
#define COMMANDER_HEADER 0x30 
#define COMMAND_INTERVAL 20 // 50Hz em ms

// ---------------------------------------------------------
// Estrutura de Payload
// ---------------------------------------------------------
struct CommanderPayload {
    float roll;
    float pitch;    
    float yaw;       
    uint16_t thrust; 
} __attribute__((packed)); 

// ---------------------------------------------------------
// Variáveis de Estado Internas (Escopo de Arquivo)
// ---------------------------------------------------------
static float current_roll = 0.0;
static float current_pitch = 0.0;
static float current_yaw = 0.0;
static uint16_t current_thrust = 0;

static unsigned long lastCommandTime = 0;
static bool is_ready_to_fly = false;

// ---------------------------------------------------------
// Funções Privadas
// ---------------------------------------------------------
static void send_crtp_command(float roll, float pitch, float yaw, uint16_t thrust) {
    CommanderPayload payload = {roll, pitch, yaw, thrust};
    uint8_t payload_size = sizeof(payload); 
    uint8_t checksum = 0;
    
    Serial1.write(CRTP_START_BYTE); 
    Serial1.write(CRTP_START_BYTE); 

    Serial1.write(COMMANDER_HEADER);
    checksum += COMMANDER_HEADER;    

    Serial1.write(payload_size);    
    checksum += payload_size;       

    uint8_t* payload_bytes = (uint8_t*)&payload;
    for (int i = 0; i < payload_size; i++) {    
        Serial1.write(payload_bytes[i]);
        checksum += payload_bytes[i];            
    }

    Serial1.write(checksum); 
}

static void process_rx_ready_to_fly() {
    static uint8_t rx_state = 0;
    static uint8_t calculated_checksum = 0;

    while (Serial1.available() > 0 && !is_ready_to_fly) {
        uint8_t c = Serial1.read();
        
        // DEPURAÇÃO (Descomente a linha abaixo para visualizar o raw dump de bytes chegando na UART)
        // Serial.printf("%02X ", c);

        switch (rx_state) {
            case 0: // 1. Aguarda o primeiro START_BYTE (0xAA)
                if (c == CRTP_START_BYTE) {
                    rx_state = 1;
                }
                break;
                
            case 1: // 2. Aguarda o segundo START_BYTE (0xAA)
                if (c == CRTP_START_BYTE) {
                    rx_state = 2;
                } else {
                    rx_state = 0; // Falso positivo, reseta a máquina
                }
                break;
                
            case 2: // 3. Aguarda o HEADER (0xF0)
                if (c == 0xF0) {
                    calculated_checksum = c;
                    rx_state = 3;
                } else if (c == CRTP_START_BYTE) {
                    rx_state = 1; // Mantém a sincronia se a linha estiver suja com múltiplos 0xAA
                } else {
                    rx_state = 0;
                }
                break;
                
            case 3: // 4. Aguarda o SIZE (0x01)
                if (c == 0x01) {
                    calculated_checksum += c;
                    rx_state = 4;
                } else {
                    rx_state = 0;
                }
                break;
                
            case 4: // 5. Aguarda o PAYLOAD (0x01)
                if (c == 0x01) {
                    calculated_checksum += c;
                    rx_state = 5;
                } else {
                    rx_state = 0;
                }
                break;
                
            case 5: // 6. Valida o CHECKSUM (0xF0 + 0x01 + 0x01 = 0xF2)
                if (c == calculated_checksum) {
                    is_ready_to_fly = true;
                    Serial.println("\n[RX] >>> SINAL [READY TO FLY] RECEBIDO E VALIDADO! <<<");
                    lastCommandTime = millis();
                } else {
                    // Erro detalhado para identificar bit flips ou corrupção de payload
                    Serial.printf("\n[RX ERRO] Checksum falhou. Calculado: %02X | Recebido: %02X\n", calculated_checksum, c);
                }
                rx_state = 0; // Prepara para o próximo pacote caso falhe
                break;
        }
    }
}

// ---------------------------------------------------------
// Implementação da API Pública
// ---------------------------------------------------------
void crtp_init() {
    Serial.begin(115200); 
    delay(1000);
    Serial.println("=== AGUARDANDO SINAL DE SINCRONIZACAO (READY TO FLY) ===");
    Serial1.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);
}

bool crtp_is_ready() {
    return is_ready_to_fly;
}

void TakeOff() {
    current_roll = 0.0;
    current_pitch = 0.0;
    current_yaw = 0.0;
    current_thrust = 45000;
}

void Hover() {
    current_roll = 0.0;
    current_pitch = 0.0;
    current_yaw = 0.0;
    current_thrust = 32767;
}

void Moving() {
    current_roll = 0.0;
    current_pitch = -15.0; 
    current_yaw = 0.0;
    current_thrust = 32767;
}

void Landing(unsigned long timeInState) {
    current_roll = 0.0;
    current_pitch = 0.0;
    current_yaw = 0.0;
    
    if (timeInState < 5000) {
        float progress = (float)timeInState / 5000.0;
        float reverse_progress = 1.0 - progress;
        const uint16_t MIN_LIFT_THRUST = 18000; 
        const uint16_t MAX_HOVER_THRUST = 32767;
        current_thrust = MIN_LIFT_THRUST + ((MAX_HOVER_THRUST - MIN_LIFT_THRUST) * (reverse_progress * reverse_progress));
    } else {
        current_thrust = 0;
    }
}

void crtp_update() {
    if (!is_ready_to_fly) {
        process_rx_ready_to_fly();
        return; 
    }

    unsigned long currentMillis = millis();
    
    // Controle de envio não-bloqueante a 50Hz
    if (currentMillis - lastCommandTime >= COMMAND_INTERVAL) {
        lastCommandTime = currentMillis;
        send_crtp_command(current_roll, current_pitch, current_yaw, current_thrust);
        
        // Mantém o buffer de recepção vazio enquanto opera
        while (Serial1.available()) {
            Serial1.read();
        }
    }
}