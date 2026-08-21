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

#include "Center.h"
#include "VisionSystem.h"
#include "logoSats.h"
#include <Arduino.h>

#define IMG_WIDTH 320
#define IMG_HEIGHT 240
#define CENTER_X (IMG_WIDTH / 2)
#define CENTER_Y (IMG_HEIGHT / 2)
#define MARGIN_X 80
#define MARGIN_Y 60

// Declarações externas das variáveis globais
extern Point *contour;
extern int contourSize;

static int getPositionCode(int cX, int cY) {
    int errX = cX - CENTER_X;
    int errY = CENTER_Y - cY; 
    
    // Retorna 0 quando o centroide estiver dentro da margem
    if (abs(errX) <= MARGIN_X && abs(errY) <= MARGIN_Y) {
        return 0;
    }
    if (errX >= 0 && errY >= 0) return 1;      // Q1: Superior Direito
    else if (errX < 0 && errY >= 0) return 2; // Q2: Superior Esquerdo
    else if (errX < 0 && errY < 0) return 3;  // Q3: Inferior Esquerdo
    else return 4;                             // Q4: Inferior Direito
}

static void drawPixel(uint8_t* buf, int x, int y) {
    if (x >= 0 && x < IMG_WIDTH && y >= 0 && y < IMG_HEIGHT) {
        buf[y * IMG_WIDTH + x] = 255;
    }
}

static void drawLogo(uint8_t* buf) {
    int startX = IMG_WIDTH - LOGO_WIDTH - 5; 
    int startY = IMG_HEIGHT - LOGO_HEIGHT - 5; 

    for (int y = 0; y < LOGO_HEIGHT; y++) {
        for (int x = 0; x < LOGO_WIDTH; x++) {
            int imgX = startX + x;
            int imgY = startY + y;
            
            if (imgX >= 0 && imgX < IMG_WIDTH && imgY >= 0 && imgY < IMG_HEIGHT) {
                uint8_t pixelVal = pgm_read_byte(&sats_logo[y * LOGO_WIDTH + x]);
                if (pixelVal == 1) { 
                    buf[imgY * IMG_WIDTH + imgX] = 255;
                }
            }
        }
    }
}

static void renderHUD(uint8_t* buf) {
    int x_min = CENTER_X - MARGIN_X; 
    int x_max = CENTER_X + MARGIN_X;
    int y_min = CENTER_Y - MARGIN_Y; 
    int y_max = CENTER_Y + MARGIN_Y;

    // Linhas centrais tracejadas
    drawLine(buf, IMG_WIDTH, IMG_HEIGHT, 0, CENTER_Y, x_min, CENTER_Y); 
    drawLine(buf, IMG_WIDTH, IMG_HEIGHT, x_max, CENTER_Y, IMG_WIDTH, CENTER_Y); 
    
    drawLine(buf, IMG_WIDTH, IMG_HEIGHT, CENTER_X, 0, CENTER_X, y_min);
    drawLine(buf, IMG_WIDTH, IMG_HEIGHT, CENTER_X, y_max, CENTER_X, IMG_HEIGHT);

    drawLine(buf, IMG_WIDTH, IMG_HEIGHT, x_min, y_min, x_max, y_min); 
    drawLine(buf, IMG_WIDTH, IMG_HEIGHT, x_min, y_max, x_max, y_max);
    drawLine(buf, IMG_WIDTH, IMG_HEIGHT, x_min, y_min, x_min, y_max); 
    drawLine(buf, IMG_WIDTH, IMG_HEIGHT, x_max, y_min, x_max, y_max);
    
    drawLogo(buf);
}

void initCenterBuffers() {
    initVisionBuffers();
}

int processCentralization(camera_fb_t *fb, String detectedType) {
    if (fb == NULL || fb->buf == NULL) return -1;
    int positionCode = -1;
    findContour(fb->buf, fb->width, fb->height);
    if (contourSize > 300) {
        simplifyContour(18.0);
        int centroX = 0, centroY = 0;
        calculateCentroid(&centroX, &centroY);
        if (detectedType == "QUADRADO" || detectedType == "TRIANGULO") {
            positionCode = getPositionCode(centroX, centroY);
            // Marca o centroide computado com um pequeno bloco cruzado no buffer da imagem
            for (int i = -2; i <= 2; i++) {
                for (int j = -2; j <= 2; j++) {
                    drawPixel(fb->buf, centroX + i, centroY + j);
                }
            }
        }

        // Desenha o contorno simplificado/capturado no frame para fins visuais
        if (contourSize > 1) {
            for (int i = 0; i < contourSize - 1; i++) {
                drawLine(fb->buf, fb->width, fb->height, contour[i].x, contour[i].y, contour[i+1].x, contour[i+1].y);
            }
        }
    }
    // Renderiza as margens de centralização e o logotipo HUD
    renderHUD(fb->buf);
    return positionCode;
}

void CentralizeDrone(int positionCode) {
    float target_pitch = 0.0;
    float target_roll = 0.0;

    // Mapeamento espacial para centralização
    switch (positionCode) {
        case 1: 
            // Q1: Superior Direito -> Mover para Frente e Direita
            target_pitch = -5.0; 
            target_roll = 5.0;   
            break;
        case 2: 
            // Q2: Superior Esquerdo -> Mover para Frente e Esquerda
            target_pitch = -5.0; 
            target_roll = -5.0;  
            break;
        case 3: 
            // Q3: Inferior Esquerdo -> Mover para Trás e Esquerda
            target_pitch = 5.0;  
            target_roll = -5.0;  
            break;
        case 4: 
            // Q4: Inferior Direito -> Mover para Trás e Direita
            target_pitch = 5.0;  
            target_roll = 5.0;   
            break;
        case 0:
        default:
            // Centro absoluto ou perda de tracking -> Hover
            target_pitch = 0.0;
            target_roll = 0.0;
            break;
    }
}