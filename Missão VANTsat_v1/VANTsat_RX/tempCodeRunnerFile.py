import serial
import cv2
import numpy as np
import struct

def listen_heltec_stream(porta_com, baudrate=115200):
    try:
        ser = serial.Serial()
        ser.port = porta_com
        ser.baudrate = baudrate
        ser.timeout = 0.1  
        
        # Prevenção de Reset no ESP32/Heltec via DTR/RTS[cite: 4]
        ser.dtr = False
        ser.rts = False
        ser.open()
        ser.setDTR(False)
        ser.setRTS(False)
        
        ser.flushInput()
        print(f"[PYTHON] Escutando {porta_com}...")
    except Exception as e:
        print(f"Erro ao abrir porta: {e}")
        return

    sync_marker = b'\xaa\xbb\xcc\xdd'
    buffer = bytearray()
    ultimo_log = "Aguardando log..."

    while True:
        try:
            if ser.in_waiting > 0:
                buffer.extend(ser.read(ser.in_waiting))
            
            idx = buffer.find(sync_marker)
            if idx != -1:
                buffer = buffer[idx:]
                
                # Valida se temos o tamanho do cabeçalho binário atualizado (13 bytes)
                if len(buffer) >= 13:
                    frame_type = buffer[4]
                    
                    # Extrai o ID e o Tamanho em sequência (8 bytes usando '<II')
                    frame_id, payload_size = struct.unpack('<II', buffer[5:13]) 
                    total_frame_size = 13 + payload_size + 2 # +2 bytes do SYNC_END
                    
                    if len(buffer) >= total_frame_size:
                        payload = buffer[13:13+payload_size]
                        sync_end = buffer[13+payload_size:total_frame_size]
                        
                        if sync_end == b'\xee\xff':
                            if frame_type == 0x01:
                                # --- 1. Processamento de LOG ---[cite: 4]
                                ultimo_log = payload.decode('utf-8', errors='ignore')
                                print(f"\n[LOG RECEBIDO] {ultimo_log}")
                                
                            elif frame_type == 0x02:
                                # --- 2. Processamento de IMAGEM (RAW ou JPEG) ---[cite: 4]
                                img = None
                                nparr = np.frombuffer(payload, np.uint8)
                                
                                # Verifica o 'Magic Number' do JPEG (\xff\xd8)[cite: 4]
                                if payload.startswith(b'\xff\xd8'):
                                    # Imagem original comprimida[cite: 4]
                                    img = cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)
                                else:
                                    # Imagem RAW pós-processamento (Tons de cinza)[cite: 4]
                                    if payload_size == 76800:
                                        # QVGA exato: 240 linhas por 320 colunas[cite: 4]
                                        img = nparr.reshape((240, 320))
                                    elif payload_size == 34560:
                                        img = nparr.reshape((144, 240))
                                    else:
                                        print(f"[ERRO] Formato RAW não mapeado. Tamanho: {payload_size} bytes")

                                # Exibição e Feedback Visual[cite: 4]
                                if img is not None:
                                    print(f"[FRAME RECEBIDO] Buffer de {payload_size} bytes processado com sucesso. ID: {frame_id}")
                                    # Imprime o texto sobre a imagem para vincular o log à visualização[cite: 4]
                                    cv2.putText(img, ultimo_log, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                                    cv2.imshow("VANTsat Mission View", img)
                                else:
                                    print(f"[ERRO] Falha ao construir o frame com tamanho: {payload_size}")
                        else:
                            print("[ERRO] Finalizador de pacote corrompido.")
                        
                        # Limpa o frame atual do buffer de memória[cite: 4]
                        buffer = buffer[total_frame_size:]
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            print(f"Exceção no loop: {e}")
            continue

    ser.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    listen_heltec_stream('COM8', 115200)