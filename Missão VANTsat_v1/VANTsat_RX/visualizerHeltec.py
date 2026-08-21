import serial
import cv2
import numpy as np
import struct
import time

def listen_heltec_stream(porta_com, baudrate=115200):
    try:
        ser = serial.Serial()
        ser.port = porta_com
        ser.baudrate = baudrate
        ser.timeout = 0.1  
        
        # Manter DTR e RTS em True para evitar reset no ESP32-S3
        ser.dtr = True
        ser.rts = True
        ser.open()
        
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
            # 1. Acúmulo contínuo de buffer
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                buffer.extend(chunk)
            
            # 2. Busca do marcador de sincronismo
            idx = buffer.find(sync_marker)
            if idx != -1:
                # Descarta ruídos lidos antes do início oficial do frame
                if idx > 0:
                    buffer = buffer[idx:]
                
                # Valida tamanho mínimo do cabeçalho corrigido (9 bytes)
                # Sincronismo (4) + Tipo (1) + Tamanho Payload (4) = 9 bytes
                if len(buffer) >= 9:
                    frame_type = buffer[4]
                    
                    # Extrai apenas o Tamanho (4 bytes) do payload
                    payload_size = struct.unpack('<I', buffer[5:9])[0] 
                    
                    # Proteção contra falha de decodificação
                    if payload_size > 300000: 
                        print(f"[ERRO] Tamanho de payload corrompido: {payload_size} bytes. Limpando buffer.")
                        buffer = buffer[4:]
                        continue

                    total_frame_size = 9 + payload_size + 2
                    
                    # 3. Aguarda até o buffer ter recebido o frame completo
                    if len(buffer) >= total_frame_size:
                        payload = buffer[9:9+payload_size]
                        sync_end = buffer[9+payload_size:total_frame_size]
                        
                        # 4. Validação do Terminador
                        if sync_end == b'\xee\xff':
                            
                            # Processamento de LOG
                            if frame_type == 0x01:
                                ultimo_log = payload.decode('utf-8', errors='ignore')
                                print(f"\n[LOG RECEBIDO] {ultimo_log}")
                                
                            # Processamento de IMAGEM
                            elif frame_type == 0x02:
                                img = None
                                nparr = np.frombuffer(payload, np.uint8)
                                
                                # Detecção de Magic Number JPEG (\xff\xd8)
                                if payload.startswith(b'\xff\xd8'):
                                    img = cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)
                                else:
                                    # Mapeamento RAW
                                    if payload_size == 76800:
                                        img = nparr.reshape((240, 320))
                                    elif payload_size == 34560:
                                        img = nparr.reshape((144, 240))
                                    else:
                                        print(f"[ERRO] RAW não mapeado: {payload_size} bytes.")

                                # Renderização Visual
                                if img is not None:
                                    print(f"[FRAME] Imagem recebida ({payload_size} bytes)")
                                    
                                    # Não adiciona informações sobre a imagem
                                    # A imagem é exibida exatamente como recebida
                                    cv2.imshow("VANTsat Mission View", img)
                                else:
                                    print(f"[ERRO] Falha de OpenCV na decodificação do frame de {payload_size} bytes.")
                        else:
                            print(f"[ERRO] Terminador inválido ({sync_end.hex()}). Buscando próximo frame.")
                            buffer = buffer[4:]
                            continue
                        
                        # Esvazia a porção lida com sucesso do buffer
                        buffer = buffer[total_frame_size:]
            
            # Saída limpa ao pressionar 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            print(f"Exceção no loop serial: {e}")
            continue

    ser.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    listen_heltec_stream('COM8', 115200)