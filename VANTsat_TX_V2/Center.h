#ifndef CENTER_H
#define CENTER_H

#include "esp_camera.h"
#include <Arduino.h>
#include <math.h>

// Inicialização da alocação de memória (deve ser chamada no setup principal)
void initCenterBuffers();

// Função de centralização: processa visão, renderiza HUD/Logo e retorna código de posição
int processCentralization(camera_fb_t *fb, String detectedType);

#endif // CENTER_H