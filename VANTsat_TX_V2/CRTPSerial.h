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

#endif // CRTP_TRANSMITTER_H