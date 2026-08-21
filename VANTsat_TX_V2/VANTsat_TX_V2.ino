//Bibliotecas
#include "esp_camera.h"
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include "CRTPSerial.h"

#include "VisionSystem.h"
#include "NetworkManager.h"
#include "StorageHandler.h"
#include "Center.h"

// --- DEFINIÇÃO DE PINOS XIAO ESP32S3 SENSE ---
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

// --- PINOS SPI PARA O CARTÃO MICROSD ---
#define SD_SCK  7
#define SD_MISO 8
#define SD_MOSI 9
#define SD_CS   21

// --- CONTROLE TEMPORAL DA MISSÃO AUTÔNOMA ---
unsigned long lastCaptureTime = 0;
const unsigned long CAPTURE_INTERVAL_MS = 5000; // Define o intervalo em milissegundos (10s)
unsigned long testSequenceStartTime = 0;

// --- CONTROLE TEMPORAL DA MISSÃO CRTP ---
unsigned long missionStartTime = 0;
unsigned long landingStartTime = 0;
uint8_t currentTestStage = 0;
bool testStarted = false;
bool missionFinished = false;


void setup() {
    Serial.begin(115200);

    // 1. Inicializa o Cartão SD
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if(!SD.begin(SD_CS, SPI)) { 
        Serial.println("[ERR] Falha ao montar SD");
    } else {
        Serial.println("[OK] SD montado com sucesso.");
    }
    
    // 2. Inicializa a PSRAM
    if (psramInit()) {
        initVisionBuffers();
        Serial.println("[OK] PSRAM Inicializada");
    } else {
        Serial.println("[ERR] PSRAM não encontrada! O sistema pode falhar.");
    }
    
    // 3. Configuração da Câmera
    camera_config_t config;
    memset(&config, 0, sizeof(camera_config_t)); 

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM; 
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM; 
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM; 
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM; 
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM; 
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000; 
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_PSRAM;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[ERR] Inicialização da câmera falhou com erro 0x%x", err);
        return;
    }

    sensor_t * s = esp_camera_sensor_get();
    if (s) {
        // --- AJUSTES PARA AMBIENTES COM MUITO BRILHO ---
        
        s->set_exposure_ctrl(s, 1);      // 1 = Mantém automático para se adaptar
        s->set_ae_level(s, -2);          // FORÇA a exposição para o mínimo (-2 a 2)
        
        s->set_gain_ctrl(s, 0);          // DESLIGA o ganho automático 
        s->set_agc_gain(s, 0);           // Define o ganho no mínimo manual (0 a 30)
        s->set_gainceiling(s, (gainceiling_t)0); // Garante que o teto do ganho seja o menor

        // --- AJUSTES DE IMAGEM PARA RECONHECIMENTO ---
        
        s->set_brightness(s, -1);        // Reduz o brilho base
        s->set_contrast(s, 2);           // Aumenta o contraste
        s->set_sharpness(s, 2);          // Mantém as bordas nítidas
        
        // Ajuste de Branco (WB)
        s->set_whitebal(s, 1);           // Ativa Balanço de Branco
        s->set_wb_mode(s, 1);            // 1 = Modo ensolarado (Sunny)
        
        s->set_bpc(s, 1);                // Correção de pixels pretos
        s->set_wpc(s, 1);                // Correção de pixels brancos (ajuda com reflexos)
    }    

    // 4. Inicializa o Access Point WiFi
    setupWiFi();

    initCenterBuffers();

    // 5. Configura o tempo inicial da missão
    missionStartTime = millis();
    lastCaptureTime = millis(); // Adicione esta linha para sincronizar o timer
    // Inicializa a interface Serial1 para o CRTP
    crtp_init();
    testSequenceStartTime = millis();

    Serial.println("\n--- VANTsat TX (XIAO ESP32S3 SENSE): ONLINE ---");
}


void loop() {
    // A malha de controle CRTP só opera durante a missão
    if (!missionFinished) {
        
        // 1. Manutenção do CRTP durante a missão ativa
        crtp_update();

        // 2. Aguarda sinal Ready to Fly
        if (!crtp_is_ready()) {
            return; 
        }

        // 3. Inicialização e Decolagem
        if (!testStarted) {
            resetMission();
            TakeOff(); // Define current_thrust = 45000

            unsigned long takeoff_start = millis();
            while (millis() - takeoff_start < 5000) {
                crtp_update(); // Mantém o envio de pacotes a 50Hz
            } 
            testStarted = true;
        }

        // 4. Captura Inicial 
        String imageFound = captureAndSave();

        // 5. Laço de Execução de Visão
        while (imageFound != "TRIANGULO" && imageFound != "QUADRADO") {
            crtp_update();
            
            Hover(); // Define current_thrust = 32767

            unsigned long hover_start = millis();
            while (millis() - hover_start < 500) { 
                crtp_update();
            } // Define current_thrust = 32767
            imageFound = captureAndSave(); 

            if (imageFound == "TRIANGULO" || imageFound == "QUADRADO") {
                break;
            }
        }
        // 6. Execução das Ações de Missão
        if (imageFound == "TRIANGULO") {
            Moving(); // Define pitch = -15.0
            
            // Retenção de estado de movimento durante 5000 ms
            unsigned long moving_start = millis();
            while (millis() - moving_start < 5000) {
                crtp_update(); 
            } 
        } else if (imageFound == "QUADRADO") {

            unsigned long landing_start_time = millis(); 
            unsigned long timeInState = 0;

            // Retenção do estado de Pouso por 5 segundos
            while (timeInState < 5000) {
                crtp_update();
                timeInState = millis() - landing_start_time;
                Landing(timeInState); // Decaimento progressivo do current_thrust
            }

            // Encerra a missão, o que bloqueia permanentemente chamadas futuras a crtp_update()
            missionFinished = true;
            
            // Encerramento do barramento de hardware UART (RX/TX)
            Serial1.end(); 

            Serial.println("[MISSÃO] Quadrado detectado. Pousando e barramento Serial1 (CRTP) encerrado.");
            
            // Inicializa a rede para telemetria/acesso web
            setupWiFi(); 
        }
    }
    // 7. Manutenção de Rede em Estado de Repouso
    if (missionFinished) {
          handleClient(); 
    }
    static unsigned long lastMemCheck = 0;
    if (millis() - lastMemCheck > 1000) {
    lastMemCheck = millis();
    }
}
