import serial
import cv2
import numpy as np
import time

# --- Configurações da Porta Serial ---
PORTA = 'COM12' 
BAUD = 115200

# --- Dimensões da imagem raw (QVGA) ---
ALTURA = 240
LARGURA = 320

def conectar_serial():
    """
    Cria a conexão serial configurando DTR e RTS ANTES de abrir a porta.
    Isso impede que o circuito de auto-reset do ESP32 reinicie a placa.
    Retorna o objeto serial ativo ou None em caso de falha.
    """
    try:
        ser = serial.Serial()     # Instancia sem abrir a porta
        ser.port = PORTA
        ser.baudrate = BAUD
        ser.timeout = 3
        
        # Configurações críticas: define o estado lógico antes de abrir a porta
        ser.dtr = False 
        ser.rts = False 
        
        ser.open()                # Abre a porta fisicamente
        time.sleep(1)             # Breve estabilização elétrica do barramento USB
        print(f"[SISTEMA] Conexão {PORTA} aberta com sucesso e mantida ativa.")
        return ser
    except Exception as e:
        print(f"[ERRO] Falha ao abrir porta serial: {e}")
        return None

def comunicar_esp(ser, comando):
    """
    Utiliza a conexão serial persistente para enviar comandos e receber respostas.
    Recebe o objeto 'ser' por injeção de dependência.
    """
    img_decodificada = None
    
    try:
        # Limpa lixo residual no buffer gerado enquanto o sistema estava ocioso
        ser.reset_input_buffer() 
        print(f"\n[TX] Enviando comando: {comando}")
        ser.write(comando.encode('utf-8'))
        
        # Lógica para Comandos de Ação Simples (Telemetria)
        if comando in ['C', 'R']:
            for _ in range(30):
                if ser.in_waiting > 0:
                    resposta = ser.readline().decode('utf-8', errors='ignore').strip()
                    if "DONE:" in resposta:
                        print(f"[TELEMETRIA] {resposta}")
                    else:
                        print(f"[RX] {resposta}")
                    break
                time.sleep(0.1)
                
        # Lógica para Download de Imagem (Comandos de 0 a 9)
        elif comando.isdigit():
            header = ""
            for _ in range(50): 
                if ser.in_waiting > 0:
                    header = ser.readline().decode('utf-8', errors='ignore').strip()
                    if header.startswith("START:"):
                        break
                time.sleep(0.1)
            
            if header.startswith("START:"):
                partes = header.split(":")
                tamanho_total = int(partes[2])
                print(f"[INFO] Recebendo imagem de {tamanho_total} bytes...")
                
                dados_brutos = bytearray()
                inicio = time.time()
                
                while len(dados_brutos) < tamanho_total:
                    faltam = tamanho_total - len(dados_brutos)
                    if ser.in_waiting > 0:
                        dados_brutos.extend(ser.read(min(ser.in_waiting, faltam)))
                    
                    if time.time() - inicio > 15:
                        print(f"[ERRO] Timeout na leitura da imagem.")
                        break
                        
                # Processamento Final da Imagem
                if len(dados_brutos) == tamanho_total:
                    time.sleep(0.05)
                    if ser.in_waiting > 0:
                        ser.read_until(b"END_FRAME\n")
                        
                    print("[OK] Imagem recebida com sucesso. Montando matriz Grayscale...")
                    nparr = np.frombuffer(dados_brutos, dtype=np.uint8)
                    
                    if len(nparr) == (ALTURA * LARGURA):
                        img_decodificada = nparr.reshape((ALTURA, LARGURA))
                    else:
                        print(f"[ERRO] Tamanho dos dados ({len(nparr)}) não corresponde a {LARGURA}x{ALTURA}.")
            else:
                print("[ERRO] Cabeçalho de imagem não encontrado. Verifique o ESP32.")
                
    except Exception as e:
        print(f"[ERRO] Falha na comunicação: {e}")
        
    return img_decodificada

def main():
    """
    Função principal que gerencia o ciclo de vida da serial e a interface com o usuário.
    """
    print("=" * 40)
    print("   VISUALIZADOR E CONTROLE VANTsat   ")
    print("=" * 40)
    print("COMANDOS:\n[C] Capturar Foto\n[R] Resetar Missão\n[0-9] Baixar Foto do SD\n[Q] Sair")
    
    # 1. Abre a conexão uma única vez no início do programa
    ser = conectar_serial()
    if not ser:
        return # Encerra o script se não conseguir abrir a porta

    while True:
        cmd = input("\nDigite o comando >> ").strip().upper()
        
        if cmd == 'Q':
            print("Encerrando o sistema...")
            break
            
        elif cmd in ['C', 'R'] or (cmd.isdigit() and 0 <= int(cmd) <= 9):
            # 2. Passa o objeto serial aberto para a função de comunicação
            img = comunicar_esp(ser, cmd)
            
            if img is not None:
                cv2.imshow("VANTsat - Visualizador", img)
                cv2.waitKey(1)
        else:
            print("[!] Comando não reconhecido. Tente novamente.")
            
    # 3. Liberação de Hardware ao sair do loop principal
    if ser and ser.is_open:
        ser.close()
        print(f"[SISTEMA] Conexão {PORTA} fechada com segurança.")
        
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()