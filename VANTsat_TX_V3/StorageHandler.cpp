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
 * CONTEXTO: Responsável pela persistência de dados em memória não volátil 
 *           (SD). Atua em conjunto com o `VisionSystem` e o `Center` para o registro 
 *           estruturado de logs de telemetria e metadados das inferências.
 **************************************************************************************************/

#include "StorageHandler.h"
#include "VisionSystem.h"
#include "esp_camera.h"
#include "Center.h"

#define MISSION_DIR "/missao"
#define LOG_FILE "/missao/data.txt"

// Inicialização das variáveis de estado
const int MAX_CIRCULAR_INDEX = 20;
int circularIndex = 0;
uint32_t totalIndex = 1;
extern unsigned long missionStartTime;
int droneControlPositionCode = -1;

// ---------------------------------------------------------
// Capitura e Salva as Fotos no SD
// ---------------------------------------------------------
// NÃO ALTERE NADA AQUI!
String captureAndSave() {
    camera_fb_t * old_fb = esp_camera_fb_get();
    if (old_fb) {
        esp_camera_fb_return(old_fb);
    }

    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("ERR:CAMERA_FAIL");
        return "";
    }

    String tipoFigura = processVisionFrame(fb);
 
    if (tipoFigura != "TRIANGULO" && tipoFigura != "QUADRADO") {
        esp_camera_fb_return(fb);
        return "DESCONHECIDO";
    }

    droneControlPositionCode = processCentralization(fb, tipoFigura);
    Serial.printf("Quadrante: %d\n", droneControlPositionCode);
    
    String path = String(MISSION_DIR) + "/" + String(circularIndex) + ".jpg";

    size_t jpeg_len = 0;
    uint8_t * jpeg_buf = NULL;

    bool jpeg_converted = fmt2jpg(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_GRAYSCALE, 80, &jpeg_buf, &jpeg_len);

    if (!jpeg_converted) {
        Serial.println("ERR:JPEG_COMPRESSION_FAIL");
        esp_camera_fb_return(fb);
        return "DESCONHECIDO";
    }
    if (!SD.exists(MISSION_DIR)) {
        SD.mkdir(MISSION_DIR);
    }

    File file = SD.open(path.c_str(), FILE_WRITE);
    size_t bytesSalvos = 0; 

    if (!file) {
        Serial.println("ERR:SD_WRITE_FAIL");
    } else {
        size_t bytes_remaining = jpeg_len;
        uint8_t * write_ptr = jpeg_buf;
        const size_t CHUNK_SIZE = 4096; 

        while (bytes_remaining > 0) {
            size_t to_write = (bytes_remaining > CHUNK_SIZE) ? CHUNK_SIZE : bytes_remaining;
            size_t written = file.write(write_ptr, to_write);
            
            bytesSalvos += written;
            write_ptr += written;
            bytes_remaining -= written;

            if (written != to_write) {
                Serial.println("ERR:SD_WRITE_TRUNCATED");
                break; 
            }
        }
        file.close();

        double missionTimeSeconds = (millis() - missionStartTime) / 1000.0;
        File log = SD.open(LOG_FILE, FILE_APPEND);
        if (log) {
            log.printf("TIPO:%s, CID:%d, TID:%u, Size:%zu, TS:%.3f\n", 
                        tipoFigura.c_str(), circularIndex, totalIndex, bytesSalvos, missionTimeSeconds);
            log.close();
        }
        Serial.printf("DONE:CAPTURED:%s (FORMA:%s) [JPEG SIZE: %zu]\n", path.c_str(), tipoFigura.c_str(), bytesSalvos);
        circularIndex = (circularIndex + 1) % MAX_CIRCULAR_INDEX;
        totalIndex++;
    }
    if (jpeg_buf) {
        free(jpeg_buf);
    }
    esp_camera_fb_return(fb);
    return tipoFigura;
}

// ---------------------------------------------------------
// Reset das imagens de dados da missão
// ---------------------------------------------------------
void resetMission() {
    File dir = SD.open(MISSION_DIR);
    
    if (!dir || !dir.isDirectory()) {
        Serial.println("ERR:MISSION_DIR_NOT_FOUND_OR_INVALID");
        return;
    }
    File file = dir.openNextFile();
    while (file) {
        if (!file.isDirectory()) {
            String fileName = file.name();
            int slashIndex = fileName.lastIndexOf('/');
            if (slashIndex != -1) {
                fileName = fileName.substring(slashIndex + 1);
            }
            String filePath = String(MISSION_DIR) + "/" + fileName;
            file.close(); 
            if (SD.remove(filePath)) {
                Serial.printf("DONE:FILE_REMOVED:%s\n", filePath.c_str());
            } else {
                Serial.printf("ERR:FILE_REMOVE_FAIL:%s\n", filePath.c_str());
            }
        } else {
            file.close();
        }
        file = dir.openNextFile();
    }
    dir.close();
    // Recriação/Zera o arquivo de log
    File log = SD.open(LOG_FILE, FILE_WRITE);
    if (log) {
        log.close();
    }
    Serial.println("DONE:MISSION_RESET_COMPLETE");
}

// ---------------------------------------------------------
// Envia as fotos para Computador de Bordo
// ---------------------------------------------------------
// NÃO ALTERE NADA AQUI!
void sendImageToClient(WiFiClient &client, int index) {
    // 1. Otimização O(1): Acesso direto ao path em vez de varrer a tabela FAT
    String path = "/missao/" + String(index) + ".jpg";
    File file = SD.open(path.c_str(), FILE_READ);

    if (!file) {
        client.println("ERR:NOT_FOUND");
        return;
    }

    size_t size = file.size();
    uint32_t retTID = 0;
    float retTS = 0.0;
    char retTipo[32] = "N/A";

    // 2. Extração da telemetria a partir de log linear
    File log = SD.open("/missao/data.txt", FILE_READ);
    if (log) {
        String searchTag = "CID:" + String(index) + ",";
        while (log.available()) {
            String line = log.readStringUntil('\n');
            if (line.indexOf(searchTag) != -1) {
                int tipoStart = line.indexOf("TIPO:") + 5;
                int tipoEnd = line.indexOf(",", tipoStart);
                if (tipoStart >= 5 && tipoEnd != -1) {
                    String tipo = line.substring(tipoStart, tipoEnd);
                    strncpy(retTipo, tipo.c_str(), sizeof(retTipo) - 1);
                    retTipo[sizeof(retTipo) - 1] = '\0'; // Garantia de terminação nula
                }
                int tidStart = line.indexOf("TID:") + 4;
                int tidEnd = line.indexOf(",", tidStart);
                if (tidStart >= 4 && tidEnd != -1) {
                    retTID = line.substring(tidStart, tidEnd).toInt();
                }
                int tsStart = line.indexOf("TS:") + 3;
                if (tsStart >= 3) {
                    retTS = line.substring(tsStart).toFloat();
                }
            }
        }
        log.close();
    }

    // 3. Cabeçalho do Protocolo
    client.printf("START:%s:%d:%zu:%u:%.3f\n", retTipo, index, size, retTID, retTS);
    client.flush();

    // 4. Buffer redimensionado para 2048 bytes a fim de saturar melhor a janela TCP e reduzir syscalls
    uint8_t buffer[2048]; 
    bool transferComplete = true;

    while (file.available()) {
        if (!client.connected()) {
            Serial.println("[ERRO] Socket TCP fechado de forma assíncrona pelo cliente.");
            transferComplete = false;
            break;
        }

        size_t len = file.read(buffer, sizeof(buffer));
        size_t written = 0;
        unsigned long blockStart = millis();

        while (written < len) {
            size_t w = client.write(buffer + written, len - written);
            if (w > 0) {
                written += w;
                blockStart = millis();
            } else {
                if (millis() - blockStart > 3000) {
                    Serial.println("[ERRO] Timeout de bloqueio na escrita do TCP.");
                    transferComplete = false;
                    break;
                }
            }
            yield(); 
        }

        if (!transferComplete) break;
    }
    
    file.close();
    
    if (transferComplete && client.connected()) {
        // CORREÇÃO CRÍTICA: \n prepended isola a string END_FRAME do array de bytes JPEG para o parser receptor funcionar
        client.print("\nEND_FRAME\n");
        client.flush();
        Serial.printf("[OK] Foto %d enviada (%zu bytes) | Tipo: %s\n", index, size, retTipo);
    } else {
        Serial.printf("[FALHA] Transmissao da foto %d abortada pelo stack TCP.\n", index);
    }
}

void sendImageDataSerial(int index) {
    String path = "/" + String(index) + ".jpg";
    // Alterado de SD_MMC para SD
    File file = SD.open(path.c_str(), FILE_READ);
    
    if (!file) {
        Serial.println("ERR:NOT_FOUND");
        return;
    }

    size_t size = file.size();
    
    String missionData = "TIPO:NENHUM, CID:" + String(index) + ", TID:0, Size:0, TS:0.000";
    
    // Alterado de SD_MMC para SD
    File log = SD.open(LOG_FILE, FILE_READ);
    if (log) {
        String searchTag = "CID:" + String(index) + ",";
        while (log.available()) {
            String line = log.readStringUntil('\n');
            if (line.indexOf(searchTag) != -1) {
                missionData = line; 
            }
        }
        log.close();
    }

    Serial.printf("START_STORAGE:%d:%zu:%s\n", index, size, missionData.c_str());
    Serial.flush();
    
    uint8_t buffer[1024];
    while (file.available()) {
        int bytesRead = file.read(buffer, sizeof(buffer));
        Serial.write(buffer, bytesRead);
    }
    file.close();
    Serial.print("\nEND_FRAME\n");
    Serial.flush();
}