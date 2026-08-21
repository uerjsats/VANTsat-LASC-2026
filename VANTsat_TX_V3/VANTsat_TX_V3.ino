/**************************************************************************************************
 __     ___    _   _ _____          _             _   _ _____ ____     _           _       
 \ \   / / \  | \ | |_   _|__  __ _| |_          | | | | ____|  _ \   | |___  __ _| |_ ___ 
  \ \ / / _ \ |  \| | | |/ __|/ _` | __|  _____  | | | |  _| | |_) |  | / __|/ _` | __/ __|
   \ V / ___ \| |\  | | |\__ \ (_| | |_  |_____| | |_| | |___|  _ < |_| \__ \ (_| | |_\__ \
    \_/_/   \_\_| \_| |_||___/\__,_|\__|          \___/|_____|_| \_\___/|___/\__,_|\__|___/
                                                                                               
 **************************************************************************************************
 * PROJETO: VANTsat - Equipe UERJsats: Missão Atlas LASC 2026
 * DESENVOLVEDORES: Carlos Leal e Vitor Forny
 * GITHUB: https://github.com/uerjsats https://github.com/Caduleal https://github.com/vitorforny04
 * OBJETIVO:  Sistema de reconhecimento computacional de imagens focado na detecção 
 *          e classificação autônoma de formas geométricas (triângulos e quadrados).
 * CONTEXTO: Integração de visão (VisionSystem), rede (NetworkManager), 
 *           armazenamento (StorageHandler) e comunicação serial (CRTPSerial).
 **************************************************************************************************/

//Bibliotecas
#include "esp_camera.h"
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include "CRTPSerial.h"

//Arquivos 
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

// --- CONTROLE TEMPORAL DA MISSÃO  ---
unsigned long lastCaptureTime = 0;
unsigned long testSequenceStartTime = 0;
unsigned long missionStartTime = 0;
unsigned long landingStartTime = 0;
uint8_t currentTestStage = 0;
bool testStarted = false;
bool missionFinished = false;


void setup() {
    Serial.begin(115200);
    // Inicializa o Cartão SD
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if(!SD.begin(SD_CS, SPI)) { 
        Serial.println("[ERR] Falha ao montar SD");
    } else {
        Serial.println("[OK] SD montado com sucesso.");
    }
    // Inicializa a PSRAM
    // Verifique se a PSRAM está habilitada no Arduino IDE em "Ferramentas"
    if (psramInit()) {
        initVisionBuffers();
        Serial.println("[OK] PSRAM Inicializada");
    } else {
        Serial.println("[ERR] PSRAM não encontrada! O sistema pode falhar.");
    }
    // Configuração da Câmera
    // As Configurações de Câmera são baseadas nas lentes do ESP AI-Thinker
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
        s->set_exposure_ctrl(s, 1);      
        s->set_ae_level(s, -2);          
        s->set_gain_ctrl(s, 0);          
        s->set_agc_gain(s, 0);           
        s->set_gainceiling(s, (gainceiling_t)0); 
        // --- AJUSTES DE IMAGEM PARA RECONHECIMENTO ---
        s->set_brightness(s, -1);        // Reduz o brilho base
        s->set_contrast(s, 2);           // Aumenta o contraste
        s->set_sharpness(s, 2);          // Mantém as bordas nítidas
        // Ajuste de Branco (WB)
        s->set_whitebal(s, 1);           
        s->set_wb_mode(s, 1);            
        s->set_bpc(s, 1);                
        s->set_wpc(s, 1);                
    }
    initCenterBuffers();

    // Configura o tempo inicial da missão
    missionStartTime = millis();
    lastCaptureTime = millis(); // Adicione esta linha para sincronizar o timer
    // Inicializa a interface Serial para o CRTP
    crtp_init();
    testSequenceStartTime = millis();
    Serial.println("\n--- VANTsat TX (XIAO ESP32S3 SENSE): ONLINE ---");
}


void loop() {
    if (!missionFinished) {
        crtp_update();// Manutenção do CRTP durante a missão
        // Aguarda sinal Ready to Fly
        if (!crtp_is_ready()) {
            //return; 
        }
        // ---------------------------------------------------------
        // Inicialização e Decolagem
        // ---------------------------------------------------------
        if (!testStarted) {
            resetMission();// Apaga dos dados da missão anterior no SD
            TakeOff();// Modo inicialização de voo thrust = 45000

            // 5000ms para envio de CRTP
            unsigned long takeoff_start = millis();
            while (millis() - takeoff_start < 5000) {
                crtp_update();// Mantém o envio de pacotes a 50Hz
            }
            testStarted = true;
        }
        // Captura Inicial 
        String imageFound = captureAndSave();

        // ---------------------------------------------------------
        // Laço de estabilização (hover)
        // ---------------------------------------------------------
        while (imageFound != "TRIANGULO" && imageFound != "QUADRADO") {
            Hover(); // Define current_thrust = 32767

            Serial.println("Hover");
            unsigned long hover_start = millis();
            while (millis() - hover_start < 500) { 
                crtp_update();
            } // Define current_thrust = 32767
            imageFound = captureAndSave(); 
            if (imageFound == "TRIANGULO" || imageFound == "QUADRADO") {
                break;
            }
        }
        // ---------------------------------------------------------
        // NOVO: Laço de Centralização 
        // ---------------------------------------------------------
        if (imageFound == "TRIANGULO" || imageFound == "QUADRADO") {
            
            int posCode = -1;
            
            // 1. Obtém o ponteiro do frame buffer diretamente da câmera
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb != NULL) {
                posCode = processCentralization(fb, imageFound);
                esp_camera_fb_return(fb); // Devolve o buffer imediatamente após o uso
            }
            
            
            // Loop de correção até zerar o erro espacial (posCode == 0)
            while (posCode != 0 && posCode != -1) {
                CentralizeDrone(posCode);
                Serial.println("Centralização");
                
                unsigned long centralize_start = millis();
                while (millis() - centralize_start < 300) {
                    crtp_update(); 
                }
                
                // 2. Captura novo frame para recalcular e fechar a malha
                fb = esp_camera_fb_get();
                if (fb != NULL) {
                    posCode = processCentralization(fb, imageFound);
                    esp_camera_fb_return(fb); 
                }
            }
            
            Hover();
            unsigned long stop_inertia_start = millis();
            while (millis() - stop_inertia_start < 500) {
                crtp_update();
            }
        }
        // ---------------------------------------------------------
        // Execução dos comandos CRTP
        // ---------------------------------------------------------
        if (imageFound == "TRIANGULO") {
            Moving(); // Define pitch = -15.0
            
            // 5000ms para envio de CRTP
            unsigned long moving_start = millis();
            while (millis() - moving_start < 5000) {
                crtp_update(); 
            } 
        } else if (imageFound == "QUADRADO") {
            unsigned long landing_start_time = millis(); 
            unsigned long timeInState = 0;
            Serial.println("Landing");

            // 5000ms para envio de CRTP
            while (timeInState < 5000) {
                timeInState = millis() - landing_start_time;
                Landing(timeInState); // Decaimento progressivo do current_thrust
                crtp_update();
            }
            missionFinished = true;// Encerra a missão
            Serial1.end(); // Encerramento do barramento de hardware UART (RX/TX)
            Serial.println("[MISSÃO] Quadrado detectado. Pousando e barramento Serial1 (CRTP) encerrado.");
            setupWiFi();// Inicializa a rede para telemetria/acesso web
        }
    }
    // ---------------------------------------------------------
    // Inicialização da rede após missão finalizada
    // ---------------------------------------------------------
    if (missionFinished) {
          handleClient(); 
    }
    static unsigned long lastMemCheck = 0;
    if (millis() - lastMemCheck > 1000) {
        lastMemCheck = millis();
    }
}
