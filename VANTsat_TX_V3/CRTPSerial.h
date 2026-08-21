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
 * CONTEXTO: Responsável pela comunicação serial. Armazena os parâmetros de controle
 *           de voo. 
 **************************************************************************************************/
#ifndef CRTPSERIAL_H
#define CRTPSERIAL_H

#include <Arduino.h>
#include <stdint.h>

// Inicializa a UART e as configurações iniciais
void crtp_init();

// Retorna o status da flag de sincronização
bool crtp_is_ready();

// Deve ser chamada no loop() principal para garantir o envio a 50Hz e leitura RX
void crtp_update();

// Funções de comando de voo - alteram os setpoints atuais
void TakeOff();
void Hover();
void Moving();
void Landing(unsigned long timeInState);

#endif 