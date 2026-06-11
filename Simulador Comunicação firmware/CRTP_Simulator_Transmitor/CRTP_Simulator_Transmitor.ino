#include <Arduino.h> 
// ---------------------------------------------------------
// Definições de Hardware e UART
// ---------------------------------------------------------
#define TX_PIN        43       
#define RX_PIN        44       
#define BAUDRATE      115200   

#define CRTP_START_BYTE  0xAA
#define COMMANDER_HEADER 0x30 
// ---------------------------------------------------------
// Estrutura de Dados do Movimento (Payload)
// ---------------------------------------------------------
struct CommanderPayload {
    float roll;     
    float pitch;    
    float yaw;       
    uint16_t thrust; 
} __attribute__((packed)); 

// ---------------------------------------------------------
// Variáveis de Frequência e Tempo
// ---------------------------------------------------------
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_INTERVAL = 20; // 50Hz (20ms) - Frequência necessária para manter o voo

unsigned long stateStartTime = 0;
const unsigned long STATE_DURATION = 10000;
// ---------------------------------------------------------
// Máquina de Estados
// ---------------------------------------------------------
enum FlightState {
    TAKEOFF,
    HOVER_1,
    MOVING_1,
    HOVER_2,
    MOVING_2,
    HOVER_3,
    LANDING
};

FlightState currentState = TAKEOFF;
// Variáveis atuais a serem enviadas
float current_roll = 0.0;
float current_pitch = 0.0;
float current_yaw = 0.0;
uint16_t current_thrust = 0;

// ---------------------------------------------------------
// Função de Transmissão CRTP
// ---------------------------------------------------------
void send_crtp_command(float roll, float pitch, float yaw, uint16_t thrust) {
    CommanderPayload payload;
    payload.roll = roll;      
    payload.pitch = pitch;    
    payload.yaw = yaw;        
    payload.thrust = thrust;  

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

// ---------------------------------------------------------
// SETUP
// ---------------------------------------------------------
void setup() {
    Serial.begin(115200); 
    delay(1000);          
    Serial.println("=== INICIANDO TRANSMISSOR DE VOO CONTINUO CRTP ===");

    Serial1.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);
    
    // Inicia a contagem de tempo
    stateStartTime = millis();
}

// ---------------------------------------------------------
// LOOP - Sequência Contínua Sem Bloqueios
// ---------------------------------------------------------
void loop() {
    unsigned long currentMillis = millis();
    unsigned long timeInState = currentMillis - stateStartTime;

    if (timeInState >= STATE_DURATION) {
        stateStartTime = currentMillis; // Reinicia o cronómetro para o próximo estado
        timeInState = 0;

        switch(currentState) {
            case TAKEOFF:  currentState = HOVER_1;  Serial.println("-> HOVER_1"); break;
            case HOVER_1:  currentState = MOVING_1; Serial.println("-> MOVING_1"); break;
            case MOVING_1: currentState = HOVER_2;  Serial.println("-> HOVER_2"); break;
            case HOVER_2:  currentState = MOVING_2; Serial.println("-> MOVING_2"); break;
            case MOVING_2: currentState = HOVER_3;  Serial.println("-> HOVER_3"); break;
            case HOVER_3:  currentState = LANDING;  Serial.println("-> LANDING"); break;
            case LANDING:  currentState = TAKEOFF;  Serial.println("-> TAKEOFF"); break;
        }
    }

    // 2. Definir os valores dos comandos de acordo com o estado atual
    switch(currentState) {
        case TAKEOFF:
            current_roll = 0.0;
            current_pitch = 0.0;
            current_yaw = 0.0;
            current_thrust = 45000;
            break;
            
        case HOVER_1:
        case HOVER_2:
        case HOVER_3:
            current_roll = 0.0;
            current_pitch = 0.0;
            current_yaw = 0.0;
            current_thrust = 32767;
            break;
            
        case MOVING_1:
        case MOVING_2:
            current_roll = 0.0;
            current_pitch = -15.0; // Inclina o nariz para baixo para avançar [cite: 19]
            current_yaw = 0.0;
            current_thrust = 32767;
            break;
            
        case LANDING:
            current_roll = 0.0;
            current_pitch = 0.0;
            current_yaw = 0.0;
            
            if (timeInState < 5000) {
                // 1. Calcula o progresso do estado em uma escala normalizada de 0.0 a 1.0
                float progress = (float)timeInState / 5000.0;
                
                // 2. Inverte o progresso (inicia em 1.0 e decai até 0.0)
                float reverse_progress = 1.0 - progress;
                
                // 3. Define o limiar inferior onde os motores perdem efetividade.
                // Abaixo deste valor o drone cai. (Ajuste fino de acordo com a bateria/peso).
                const uint16_t MIN_LIFT_THRUST = 18000; 
                const uint16_t MAX_HOVER_THRUST = 32767;
                
                // 4. Aplica o decaimento quadrático (reverse_progress^2).
                current_thrust = MIN_LIFT_THRUST + ((MAX_HOVER_THRUST - MIN_LIFT_THRUST) * (reverse_progress * reverse_progress));
                
            } else {
                // 5. Corta a energia integralmente apenas no término dos 5 segundos
                current_thrust = 0;
            }
            break;
    }
    // 3. Enviar o comando constantemente a 50Hz (a cada 20 milissegundos)
    if (currentMillis - lastCommandTime >= COMMAND_INTERVAL) {
        lastCommandTime = currentMillis;
        send_crtp_command(current_roll, current_pitch, current_yaw, current_thrust);
        
        // Esvazia buffer do hardware para evitar transbordamento pelos ACKs recebidos
        while (Serial1.available()) {
            Serial1.read();
        }
    }
}