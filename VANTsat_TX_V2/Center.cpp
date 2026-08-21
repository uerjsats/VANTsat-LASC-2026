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

// Declarações externas das variáveis globais gerenciadas em VisionSystem.cpp
extern Point *contour;
extern int contourSize;

static int getPositionCode(int cX, int cY) {
    int errX = cX - CENTER_X;
    int errY = CENTER_Y - cY; // Inverte Y para adequar ao plano cartesiano (Y cresce para baixo na tela)
    
    // Retorna 0 quando o centroide estiver dentro da margem de tolerância central
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

    // Linhas centrais tracejadas (simuladas via Bresenham no VisionSystem ou diretas)
    // Aqui redesenhamos apenas o HUD visual sobre a imagem processada
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
    // Inicializa os buffers globais da visão se ainda não estiverem alocados
    initVisionBuffers();
}

int processCentralization(camera_fb_t *fb, String detectedType) {
    if (fb == NULL || fb->buf == NULL) return -1;

    int positionCode = -1; // -1 indica erro ou figura não identificada

    // Executa a varredura e o rastreamento utilizando a engine pronta do VisionSystem
    findContour(fb->buf, fb->width, fb->height);

    if (contourSize > 300) {
        simplifyContour(18.0); // Aplica RDP
        
        int centroX = 0, centroY = 0;
        calculateCentroid(&centroX, &centroY);

        // Se a forma foi validada externamente ou detectada com sucesso
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