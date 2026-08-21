# VANTsat Flight Controller & Vision System (XIAO ESP32S3 SENSE)

<div align="center">
  <img src="./VANTsat_TX_V3/Imagens%20VANTsat%20LASC%202026/Drone.gif" alt="Visualização do Drone" width="400"/>
</div>

Este projeto implementa o núcleo de processamento de imagem, telemetria, gestão de armazenamento e controle de voo para a missão **VANTsat**. O firmware foi atualizado para o hardware **XIAO ESP32S3 SENSE**, utilizando uma arquitetura modular otimizada para processamento digital de sinais em tempo real, controle dinâmico de atitude e persistência de dados em sistemas de arquivos embarcados.

## 📥 Como Clonar o Repositório e Iniciar

Para obter o código-fonte deste projeto em sua máquina local, utilize o comando de clone via Git:

```bash
git clone https://github.com/uerjsats/VANTsat-LASC-2026
```

**⚠️ Atenção à Versão:** A versão mais recente e atualizada do código está localizada no diretório **`VANTsat_TX_V3`**. Recomenda-se compilar e operar exclusivamente os arquivos contidos nesta pasta.

**Nota de Teste:** A condicional de sincronização física **Ready to Fly** (handshake inicial) encontra-se atualmente **desabilitada**. Esta modificação foi aplicada para permitir a execução imediata de testes de bancada e simulações do sistema de visão sem a necessidade de acoplamento com a base de voo.

**🔗 Arquitetura Dual-MCU e Compilação dos Firmwares:**
Para a correta execução da missão em voo, é imprescindível compreender a arquitetura de hardware distribuído. O sistema utiliza duas placas independentes operando em conjunto, exigindo toolchains de compilação específicos para cada domínio:

- **Firmware de Missão e Visão Computacional (Este repositório):** Embarcado no hardware **XIAO ESP32S3 SENSE**. Este núcleo é projetado para ser compilado e gravado de forma simplificada através da **Arduino IDE**.
- **Firmware de Controle de Voo e Atitude:** Embarcado em um segundo **ESP32 (XIAO)**. A comunicação inter-processadores ocorre via barramento **Serial (UART)**, utilizando uma implementação customizada do protocolo **CRTP** (*Crazyradio Real-Time Protocol*).

O firmware responsável pelo controle dinâmico da aeronave deve ser clonado do repositório oficial da equipe:
[https://github.com/uerjsats/Controle-Atitude-LASC-2026](https://github.com/uerjsats/Controle-Atitude-LASC-2026)

**Build System do Controle de Voo:** Ao contrário do sistema de visão, o código de atitude e estabilização exige o framework oficial da Espressif, o **ESP-IDF**. Após configurar a toolchain do ESP-IDF no seu terminal e navegar até o diretório do controlador de voo, execute a compilação, gravação e monitoramento da porta serial sequencialmente com os comandos:

```bash
cd <DIRETÓRIO_FIMWARE_CONTROLE_VOO>
idf.py fullclean
idf.py build
idf.py flash monitor
```
## 🚀 Arquitetura do Software

![Diagrama de Atividades VANTsat](./VANTsat_TX_V3/Imagens%20VANTsat%20LASC%202026/DiagramaAtividadesVANTsat.jpg)
![Diagrama de Atividades do Firmware](./VANTsat_TX_V3/Imagens%20VANTsat%20LASC%202026/Firmware-Atividades_Firmware.jpg)

O código segue o princípio de separação de interesses (*Separation of Concerns*), estruturado nos seguintes módulos técnicos:

### 1. Sistema de Visão Computacional e Orquestração (`VisionSystem.h/.cpp` e `Center.h/.cpp`)

<div align="center">
  <img src="./VANTsat_TX_V3/Imagens%20VANTsat%20LASC%202026/4.jpg" alt="Identificação de Triângulo" width="300"/>
  <img src="./VANTsat_TX_V3/Imagens%20VANTsat%20LASC%202026/5.jpg" alt="Identificação de Quadrado" width="300"/>
  <br/>
  <em>Registro visual do reconhecimento embarcado com bounding boxes: identificação de Triângulo (índice 4) e Quadrado (índice 5).</em>
</div>

O core analítico do projeto, atuando na extração de primitivas geométricas e orientação de navegação.
- **Segmentação e Simplificação:** Implementa o algoritmo **Moore-Neighbor** para rastreio de contornos em buffers Grayscale e utiliza o algoritmo **Ramer-Douglas-Peucker (RDP)** para redução da dimensionalidade poligonal.
- **Classificação Geométrica:** Identifica formas (triângulos e quadrados) primariamente através da verificação rigorosa de ângulos internos, garantindo robustez na detecção.
- **Centralização e HUD (`Center`):** Calcula o centroide da forma detectada para classificá-la em um dos quatro quadrantes de uma malha cartesiana (com margem de tolerância central de 80x60). Renderiza no próprio framebuffer contendo miras, eixos cartesianos e a logo da UERJsats.
- **Gestão de Memória:** Aloca buffers críticos na **PSRAM** para suportar a manipulação de matrizes de imagem sem causar transbordamento da memória SRAM interna.

### 2. Gestão de Armazenamento e Logs (`StorageHandler.h/.cpp`)
Camada de persistência e serialização de dados utilizando o barramento **SPI** para o cartão MicroSD.
- **Diretório Estruturado:** Os dados são isolados fisicamente no diretório de trabalho `/missao`.
- **Buffer Circular Expandido:** Implementa uma lógica de armazenamento cíclico ampliada para 20 imagens JPEG, sobrescrevendo automaticamente dados antigos para otimização do espaço físico.
- **Log de Missão:** Registra metadados cruciais estruturados (Tipo de forma, ID circular, ID total, Tamanho em bytes e Timestamp exato) no arquivo `data.txt` a cada captura e inferência positiva.

**Registros de Telemetria Extraídos (`data.txt`):**

<div align="center">

| Tipo da Forma | ID Circular (CID) | ID Total (TID) | Tamanho (Bytes) | Timestamp (s) |
|:---:|:---:|:---:|:---:|:---:|
| TRIANGULO | 0 | 1 | 9971 | 30.366 |
| TRIANGULO | 1 | 2 | 9534 | 36.124 |
| TRIANGULO | 2 | 3 | 9628 | 42.069 |
| TRIANGULO | 3 | 4 | 9846 | 47.801 |
| TRIANGULO | 4 | 5 | 9882 | 55.237 |
| QUADRADO | 5 | 6 | 8856 | 63.191 |

</div>

### 3. Servidor de Rede e Interface Web (`NetworkManager.h/.cpp`)

![Interface do Servidor 1](./VANTsat_TX_V3/Imagens%20VANTsat%20LASC%202026/ServidorA.jpeg)
<div align="center">
  <img src="./VANTsat_TX_V3/Imagens%20VANTsat%20LASC%202026/ServidorB.jpeg" alt="Visualização do Servidor 2" width="200" />
</div>

Módulo responsável pela extração de dados sem fio, ativado **exclusivamente após o término da missão física** para não onerar o pipeline de controle durante o voo.
- **Modo Access Point:** Configura o ESP32S3 como servidor (`VANTsat_AP`) com IP estático (`192.168.4.1`). **A senha para conexão na rede WiFi é `uerjsats123`.**
- **Servidor Web HTTP (Porta 80):** Hospeda uma interface embarcada que consome os metadados do arquivo texto e renderiza uma galeria dinâmica (HTML/CSS) atrelando visualmente a imagem processada aos dados da telemetria correspondente. **Para acessar os dados, após conectar na rede WiFi, abra a URL `http://192.168.4.1` em seu navegador ou acesse a URL `www.missao.atlas.com`**
- **Transporte TCP (Porta 8888):** Mantém suporte legado de altíssima velocidade para entrega de fluxos binários e envio de chunks TCP sob demanda.

### 4. Controle de Voo e C2 (`CRTPSerial.h/.cpp`)
Interface de Comando e Controle que conecta diretamente a visão ao controlador do drone via UART (Serial1).
- **Protocolo CRTP:** Estrutura pacotes baseados no *Crazyradio Real-Time Protocol* com cabeçalho de inicialização (`0xAA`) e validação estrita de checksum para prevenir corrupção no ar.
- **Streaming de Malha de Controle:** Transmite vetores contínuos de atitude (`roll`, `pitch`, `yaw`) e empuxo (`thrust`) a uma taxa de atualização rígida de 50Hz em operação não-bloqueante.
- **Máquina de Estados Cinematográfica:** Orquestra dinâmicas autônomas pré-programadas: `TakeOff` (decolagem balística), `Hover` (pairar), `Moving` (deslocamento via pitch em eixo Y) e `Landing` (pouso controlado por decaimento quadrático do empuxo).

## 🛠 Especificações Técnicas

- **Hardware:** XIAO ESP32S3 SENSE.
- **Sensor de Imagem:** OV2640 em resolução QVGA (320x240), Grayscale, qualidade JPEG definida como 10.
- **Armazenamento:** Cartão SD via protocolo nativo SPI (SCK: 7, MISO: 8, MOSI: 9, CS: 21).
- **Comunicação de Controle:** Barramento UART via Serial1 rodando a 115200 bps (RX: 44, TX: 43).
- **Ajustes de Lente (AI-Thinker):** Hardware forçado para maximizar features em ambientes claros (Exposure +1, AE Level -2, Brightness -1, Contraste +2, Sharpness +2).

## 🔧 Configuração e Compilação

1. **Placa na IDE Arduino:** `Seeed Studio XIAO ESP32S3`.
2. **Configuração de Flash:** Obrigatório habilitar `OPI PSRAM` no menu de Ferramentas. Partição recomendada para abrigar a stack de rede.
3. **Dependências:**
   - Driver nativo `esp_camera`.
   - Bibliotecas core do ESP32: `WiFi`, `WebServer`, `SD` e `SPI`.

## 📡 Fluxo de Processamento de Missão

![Diagrama de Atividade do Loop da Missão](./VANTsat_TX_V3/Imagens%20VANTsat%20LASC%202026/Firmware-Diagrama_Atividade_Loop_Missão.jpg)

Ao ser alimentado, o firmware opera sequencialmente através da seguinte máquina de estados:
1. **Sincronização (Ready to Fly):** Aguarda o *handshake* na camada física validando o pacote de *start* do controlador de voo base. *(Obs: Condicional atualmente desabilitada para testes).*
2. **Decolagem (TakeOff):** Reseta o sistema de arquivos limpando o SD (`resetMission()`) e envia o impulso inicial para ganhar altura, transmitindo os pacotes em lockstep de 50Hz.
3. **Aquisição e Busca:** Passa a comandar modo `Hover`. Captura o buffer da câmera sequencialmente em PSRAM, buscando identificar unicamente uma geometria classificada como `TRIANGULO` ou `QUADRADO`.
4. **Acionamento Condicional (Ação):**
   - Caso **TRIÂNGULO**: Comanda vetorização longitudinal (`Moving` com pitch -15.0) por exatos 5 segundos e volta ao estado de busca.
   - Caso **QUADRADO**: Comanda o encerramento da busca. Inicia o decaimento suave de motor via `Landing` por 5 segundos até o toque no solo.
5. **Transição de Pouso:** A comunicação Serial1 é desativada fisicamente e a missão computacional fechada. A flag de finalização acorda o `NetworkManager`, que sobe a rede WiFi e o servidor web para *download* local do material capturado pela base em terra.