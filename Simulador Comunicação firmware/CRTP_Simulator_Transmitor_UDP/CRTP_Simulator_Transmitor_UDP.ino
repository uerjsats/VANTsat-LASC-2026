#include <Arduino.h> 
#include <WiFi.h>
#include <WiFiUdp.h>

// ---------------------------------------------------------
// 1. Definições de Rede (Wi-Fi e UDP)
// ---------------------------------------------------------
const char* WIFI_SSID     = "ESP-DRONE_1CDBD474738D";
const char* WIFI_PASSWORD = "12345678";

// Configurações de rede estática
IPAddress local_IP(192, 168, 43, 100);
IPAddress gateway(192, 168, 43, 42);   
IPAddress subnet(255, 255, 255, 0);

// Destino do Drone
IPAddress target_ip(192, 168, 43, 42); 
const int UDP_TARGET_PORT = 2390;
WiFiUDP udp;

#define COMMANDER_HEADER 0x30 

// ---------------------------------------------------------
// 2. Estrutura de Dados do Movimento (Payload)
// ---------------------------------------------------------
struct CommanderPayload {
    float roll;
    float pitch;    
    float yaw;       
    uint16_t thrust; 
} __attribute__((packed)); 

// ---------------------------------------------------------
// 3. Variáveis de Frequência e Tempo
// ---------------------------------------------------------
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_INTERVAL = 20; // 50Hz (20ms) - Frequência necessária para manter o voo

unsigned long stateStartTime = 0;
const unsigned long STATE_DURATION = 10000;

// ---------------------------------------------------------
// 4. Máquina de Estados
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
// 5. Função de Transmissão CRTP via Wi-Fi (UDP)
// ---------------------------------------------------------
void send_crtp_command(float roll, float pitch, float yaw, uint16_t thrust) {
    CommanderPayload payload;
    payload.roll = roll;      
    payload.pitch = pitch;    
    payload.yaw = yaw;        
    payload.thrust = thrust;  

    uint8_t payload_size = sizeof(payload); 
    
    // Monta o buffer UDP: Header + Data + Checksum (Tamanho total = size + 2)
    uint8_t udp_buffer[payload_size + 2];
    uint8_t buffer_len = 0;

    // 1. Grava Header
    udp_buffer[buffer_len++] = COMMANDER_HEADER;
    uint8_t esp_drone_cksum = COMMANDER_HEADER;

    // 2. Grava Dados (Payload) e calcula checksum
    uint8_t* payload_bytes = (uint8_t*)&payload;
    for (int i = 0; i < payload_size; i++) {    
        udp_buffer[buffer_len++] = payload_bytes[i];
        esp_drone_cksum += payload_bytes[i];            
    }

    // 3. Grava Checksum
    udp_buffer[buffer_len++] = esp_drone_cksum; 

    // Disparo UDP para o Drone
    udp.beginPacket(target_ip, UDP_TARGET_PORT);
    udp.write(udp_buffer, buffer_len);
    udp.endPacket();
}

// ---------------------------------------------------------
// SETUP
// ---------------------------------------------------------
void setup() {
    Serial.begin(115200); 
    delay(1000);
    Serial.println("=== INICIANDO TRANSMISSOR DE VOO CONTINUO CRTP (WI-FI) ===");

    // Configuração do Wi-Fi
    if (!WiFi.config(local_IP, gateway, subnet)) {
        Serial.println("Aviso: Falha ao configurar IP Estático");
    }
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    Serial.print("Conectando ao Wi-Fi AP do Drone");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n[Wi-Fi] Conectado com sucesso!");
    Serial.print("[Wi-Fi] IP Local configurado: ");
    Serial.println(WiFi.localIP());

    // Inicia a contagem de tempo da máquina de estados
    stateStartTime = millis();
}

// ---------------------------------------------------------
// LOOP - Sequência Contínua Sem Bloqueios
// ---------------------------------------------------------
void loop() {
    unsigned long currentMillis = millis();
    unsigned long timeInState = currentMillis - stateStartTime;

    // 1. Controle de transição de estados
    if (timeInState >= STATE_DURATION) {
        stateStartTime = currentMillis;
        timeInState = 0; // Reinicia o cronómetro para o próximo estado

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
            current_pitch = -15.0; // Inclina o nariz para baixo para avançar
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

    // 3. Enviar o comando constantemente a 50Hz (a cada 20 milissegundos) via Wi-Fi
    if (currentMillis - lastCommandTime >= COMMAND_INTERVAL) {
        lastCommandTime = currentMillis;
        send_crtp_command(current_roll, current_pitch, current_yaw, current_thrust);
    }
}