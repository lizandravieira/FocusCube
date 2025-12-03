# **FOCUS CUBE: Pomodoro Inteligente com FreeRTOS e MQTT 🎲**

Este repositório contém o firmware do projeto da disciplina de **Sistemas Embarcados** (CESAR School), focado na criação de um dispositivo físico de produtividade baseado no método Pomodoro.

O **Focus Cube** é um cubo IoT inteligente que detecta sua orientação para iniciar automaticamente timers de foco ou descanso, eliminando a necessidade de usar o celular.

## **👥 Autores & Repositórios**

Este projeto é composto por três módulos principais (Firmware, Backend e Frontend). Abaixo estão os membros da equipe e os links para os repositórios:

### **👨‍💻 Equipe de Desenvolvimento**

* **André Goes** \- [GitHub](https://github.com/Nerebo)  
* **Gabriel Caetano** \- [GitHub](https://www.google.com/search?q=https://github.com/SeuUsuarioAqui)  
* **João Fittipaldi** \- [GitHub](https://www.google.com/search?q=https://github.com/SeuUsuarioAqui)  
* **Lizandra Vieira** \- [GitHub](https://www.google.com/search?q=https://github.com/lizandravieira)

### **🔗 Repositórios do Projeto**

* **Firmware (ESP32/FreeRTOS):** [Este Repositório](https://www.google.com/search?q=https://github.com/lizandravieira/FocusCube)  
* **Backend (Python/Flask):** [Link para o Repo do Backend](https://github.com/joaovfittipaldi/productive-cube)  
* **Frontend (React/Dashboard):** [Link para o Repo do Frontend](https://github.com/caetrias/focus-cube)

## **🚀 Funcionalidades Principais**

* **Detecção de Orientação:** Usa um acelerômetro (MPU6050) para saber qual face está para cima.  
* **Timers Automáticos:** Inicia contagens regressivas de 25, 30, 45 ou 60 minutos dependendo do lado.  
* **Pausa Inteligente (Smart Break):** Ao virar o cubo de cabeça para baixo, ele calcula o tempo de descanso ideal (5, 10, 15 ou 20 min) proporcionalmente ao seu último período de foco.  
* **Display Gigante:** Renderização customizada de dígitos grandes (Big Digits) em um display LCD 16x2 para fácil leitura à distância.  
* **Multitarefa Real:** Firmware construído sobre **FreeRTOS**, com tasks paralelas para Sensor, Display e Rede.  
* **Conectividade IoT:** Envia o status em tempo real via **MQTT** para um dashboard web e recebe comandos remotos.

## **🛠️ Hardware Necessário**

| Componente | Quantidade | Descrição |
| :---- | :---- | :---- |
| **ESP32 Dev Module** | 1 | Microcontrolador principal (Wi-Fi \+ Bluetooth). |
| **MPU-6050** | 1 | Acelerômetro e Giroscópio (3 Eixos). |
| **LCD 16x2 I2C** | 1 | Display de cristal líquido com interface I2C. |
| **Cabos Jumper** | 4 | Macho-Fêmea ou Macho-Macho. |
| **Protoboard/PCB** | 1 | Para montagem do circuito. |
| **Fonte/Cabo USB** | 1 | Alimentação 5V. |

### **Esquema de Ligação (I2C Bus)**

O projeto utiliza o barramento I2C compartilhado, protegido via software.

| Pino do Módulo | Conectar no ESP32 | Observação |
| :---- | :---- | :---- |
| **SDA** (MPU e LCD) | GPIO 21 | Dados I2C |
| **SCL** (MPU e LCD) | GPIO 22 | Clock I2C |
| **VCC** (MPU) | 3.3V (3V3) | Sensor de baixa tensão |
| **VCC** (LCD) | VIN (5V) | Display requer 5V para contraste ideal |
| **GND** (Ambos) | GND | Terra comum |

## **💻 Firmware e Arquitetura**

O firmware foi desenvolvido em **C++** utilizando o framework **Arduino** sobre o **PlatformIO** (VSCode).

### **Estrutura de Tasks (FreeRTOS)**

O sistema opera com 3 tarefas concorrentes:

1. **vTask\_Sensor (Produtor):** Lê o MPU6050 a cada 100ms, classifica a face com um threshold de 0.4g e envia eventos para as filas.  
2. **vTask\_Display (Consumidor/Porteiro):** Gerencia a lógica do Timer, calcula a Pausa Inteligente e desenha no LCD. É a única task com permissão de escrita no display.  
3. **vTask\_MQTT (Rede):** Mantém a conexão Wi-Fi/MQTT ativa, reconecta automaticamente e publica o estado atual.

### **Proteção de Recursos (Mutex)**

Como o MPU6050 e o LCD dividem os mesmos fios (GPIO 21/22), implementamos um **Semaphore Mutex** (mutexI2C) para evitar colisões de dados e travamentos do barramento I2C.

## **📡 Comunicação MQTT**

O cubo se comunica com o backend através de um Broker MQTT (padrão: broker.hivemq.com).

* **Tópico de Publicação:** focuscube/status  
  * Envia o estado atual: "FOCO 25", "PAUSA (5m)", "PARADO", etc.  
* **Tópico de Comando:** focuscube/comando  
  * Recebe mensagens da Web: Qualquer texto curto (ex: "ALARME", "REUNIAO") para exibir na tela, interrompendo o timer.

## **📂 Estrutura do Repositório**

.  
├── docs/                      \# Relatório Técnico (PDF) e Imagens do Projeto  
├── esp32-esp8266/             \# Código Fonte do Firmware (C++) \- Versão Final  
│   └── firmware\_focus\_cube.cpp  
├── raspberry-pi/              \# (Referência) Código do Backend/Dashboard  
├── schematics/                \# Diagramas de Circuito  
├── platformio.ini             \# Configuração do Ambiente e Dependências  
└── README.md                  \# Documentação do Projeto

## **🚀 Como Rodar o Projeto**

1. Clone este repositório.  
2. Abra a pasta no **VSCode** com a extensão **PlatformIO** instalada.  
3. Conecte seu ESP32 via USB.  
4. No arquivo src/main.cpp, configure suas credenciais Wi-Fi:  
   const char\* WIFI\_SSID \= "SUA\_REDE";  
   const char\* WIFI\_PASS \= "SUA\_SENHA";

5. Clique em **Upload** (Seta para a direita na barra inferior).

**CESAR School \- Sistemas Embarcados 2025**