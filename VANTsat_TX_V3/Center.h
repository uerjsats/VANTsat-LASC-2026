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
 * CONTEXTO: Reposável pela lógica de verificação e centralização do drone.
 **************************************************************************************************/

#ifndef CENTER_H
#define CENTER_H

#include "esp_camera.h"
#include <Arduino.h>
#include <math.h>

// Inicialização da alocação de memória (deve ser chamada no setup principal)
void initCenterBuffers();

// Função de centralização: processa visão, renderiza HUD/Logo e retorna código de posição
int processCentralization(camera_fb_t *fb, String detectedType);

void CentralizeDrone(int positionCode);

#endif