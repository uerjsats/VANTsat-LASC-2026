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
#ifndef STORAGE_HANDLER_H
#define STORAGE_HANDLER_H

#include "Arduino.h"
#include "FS.h"
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>

// Variáveis globais de controle da missão
extern const int MAX_CIRCULAR_INDEX;
extern const char* LOG_FILE;
extern int circularIndex;
extern uint32_t totalIndex;
extern unsigned long missionStartTime;

// Protótipos das funções de armazenamento
String captureAndSave();
void sendImageDataSerial(int index);
void sendImageToClient(WiFiClient &client, int index);
void resetMission();

#endif