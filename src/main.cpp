/* =================================================================
 * PROJETO: FOCUS CUBE (Step 11 - Serial Monitor "Tagarela")
 * AUTOR: (Seu Nome)
 * DESCRIÇÃO: Adicionado logs no Serial Monitor durante a contagem
 * regressiva para facilitar o debug.
 * =================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <PubSubClient.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h" 

#define SDA_PIN 21 
#define SCL_PIN 22 

// --- Configurações ---
const char* WIFI_SSID = "uaifai-tiradentes"; 
const char* WIFI_PASS = "bemvindoaocesar";   
const char* MQTT_BROKER = "broker.hivemq.com"; 
const int   MQTT_PORT = 1883;
const char* MQTT_TOPIC_STATUS = "focuscube/status"; 
const char* MQTT_TOPIC_COMMAND = "focuscube/comando"; 

Adafruit_MPU6050 mpu;
LiquidCrystal_I2C lcd(0x27, 16, 2); 
WiFiClient espClient;
PubSubClient mqttClient(espClient);

enum Face { FACE_POS_X, FACE_NEG_X, FACE_POS_Y, FACE_NEG_Y, FACE_POS_Z, FACE_NEG_Z, FACE_UNKNOWN };
enum ModeType { TYPE_FOCUS, TYPE_SMART_BREAK, TYPE_IDLE };

struct ModeConfig { const char* name; uint32_t seconds; ModeType type; };
enum DisplayMessageType { MSG_TYPE_FACE, MSG_TYPE_COMMAND };
struct DisplayMessage { DisplayMessageType type; union { Face face; char command[17]; } data; };

QueueHandle_t xDisplayQueue; 
QueueHandle_t xMQTTQueue;    
SemaphoreHandle_t mutexI2C; 

TaskHandle_t  xTaskSensor;
TaskHandle_t  xTaskDisplay;
TaskHandle_t  xTaskMQTT;    

// --- Protótipos ---
void vTask_Sensor(void *pvParameters);
void vTask_Display(void *pvParameters);
void vTask_MQTT(void *pvParameters); 
Face classifyFace(float ax, float ay, float az);
void setupWiFi();
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length); 
ModeConfig getModeForFace(Face f);
void loadCustomChars();       
void printBigDigit(int digit, int col); 

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); 
  mutexI2C = xSemaphoreCreateMutex();

  lcd.init();
  lcd.backlight(); 
  loadCustomChars(); 
  
  if (!mpu.begin()) {
    Serial.println("MPU Fail!");
    while(1) delay(100);
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  setupWiFi(); 
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback); 

  xDisplayQueue = xQueueCreate(5, sizeof(DisplayMessage)); 
  xMQTTQueue = xQueueCreate(5, sizeof(Face)); 

  xTaskCreate(vTask_Sensor, "SensorTask", 4096, NULL, 1, &xTaskSensor);
  xTaskCreate(vTask_MQTT, "MQTTTask", 4096, NULL, 2, &xTaskMQTT);
  xTaskCreate(vTask_Display, "DisplayTask", 4096, NULL, 1, &xTaskDisplay);
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }

// ================== TASKS ==================

void vTask_Sensor(void *pvParameters) {
  Face lastFace = FACE_UNKNOWN;
  Face candidateFace = FACE_UNKNOWN;
  unsigned long candidateSince = 0;
  sensors_event_t a, g, temp;
  
  for (;;) {
    if (xSemaphoreTake(mutexI2C, portMAX_DELAY) == pdTRUE) {
      mpu.getEvent(&a, &g, &temp); 
      xSemaphoreGive(mutexI2C);   
    }

    Face f = classifyFace(a.acceleration.x, a.acceleration.y, a.acceleration.z);
    unsigned long now = millis();
    
    if (f != candidateFace) { candidateFace = f; candidateSince = now; }
    
    if (f != lastFace && (now - candidateSince) >= 500) {
      lastFace = f;
      xQueueSend(xMQTTQueue, &lastFace, pdMS_TO_TICKS(10)); 
      DisplayMessage msg; msg.type = MSG_TYPE_FACE; msg.data.face = lastFace;
      xQueueSend(xDisplayQueue, &msg, pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}

void vTask_Display(void *pvParameters) {
  DisplayMessage msg; 
  bool timerRunning = false;
  uint32_t timerSecondsRemaining = 0;
  unsigned long lastTimerTick = 0;
  bool showModeScreen = false;
  unsigned long modeScreenUntil = 0;
  
  uint32_t lastFocusDuration = 25 * 60; 
  char currentModeName[17] = "OCIOSO"; 

  if (xSemaphoreTake(mutexI2C, portMAX_DELAY) == pdTRUE) {
    lcd.clear(); lcd.print("Focus Cube"); 
    lcd.setCursor(0, 1); lcd.print("Pronto!");
    xSemaphoreGive(mutexI2C);
  }

  for (;;) {
    if (xQueueReceive(xDisplayQueue, &msg, 0) == pdTRUE) {
      if (msg.type == MSG_TYPE_FACE) {
        ModeConfig newMode = getModeForFace(msg.data.face);
        
        if (newMode.type == TYPE_FOCUS) {
          lastFocusDuration = newMode.seconds;
          timerSecondsRemaining = newMode.seconds;
          strcpy(currentModeName, newMode.name);
          Serial.printf(">>> Foco Inciado: %d min\n", newMode.seconds/60);
          
        } else if (newMode.type == TYPE_SMART_BREAK) {
          Serial.printf(">>> Calculando Pausa (Base: %d min)\n", lastFocusDuration/60);
          
          if (lastFocusDuration <= 25 * 60) {
            timerSecondsRemaining = 5 * 60; 
            strcpy(currentModeName, "PAUSA (5m)");
          } else if (lastFocusDuration <= 30 * 60) {
            timerSecondsRemaining = 10 * 60; 
            strcpy(currentModeName, "PAUSA (10m)");
          } else if (lastFocusDuration <= 45 * 60) {
            timerSecondsRemaining = 15 * 60; 
            strcpy(currentModeName, "PAUSA (15m)");
          } else {
            timerSecondsRemaining = 20 * 60; 
            strcpy(currentModeName, "PAUSA (20m)");
          }
          
        } else {
           timerSecondsRemaining = 0;
           strcpy(currentModeName, "PARADO");
        }

        timerRunning = (timerSecondsRemaining > 0);
        lastTimerTick = millis(); 
        showModeScreen = true;
        modeScreenUntil = millis() + 1500; 
        
        if (xSemaphoreTake(mutexI2C, portMAX_DELAY) == pdTRUE) {
          lcd.clear(); lcd.print("Modo:"); 
          lcd.setCursor(0, 1); lcd.print(currentModeName); 
          xSemaphoreGive(mutexI2C);
        }
      } else if (msg.type == MSG_TYPE_COMMAND) {
        timerRunning = false; showModeScreen = true; modeScreenUntil = millis() + 3000;
        if (xSemaphoreTake(mutexI2C, portMAX_DELAY) == pdTRUE) {
           lcd.clear(); lcd.print("Web diz:");
           lcd.setCursor(0, 1); lcd.print(msg.data.command);
           xSemaphoreGive(mutexI2C);
        }
      }
    }

    if (timerRunning) {
      if (millis() - lastTimerTick >= 1000) {
        lastTimerTick = millis(); 
        if (timerSecondsRemaining > 0) {
          timerSecondsRemaining--; 
          
          // [NOVIDADE] Imprime no Serial a cada segundo!
          Serial.printf("[TIMER] %02d:%02d\n", timerSecondsRemaining / 60, timerSecondsRemaining % 60);
          
        } else {
          timerRunning = false; showModeScreen = true; modeScreenUntil = millis() + 5000;
          if (xSemaphoreTake(mutexI2C, portMAX_DELAY) == pdTRUE) {
            lcd.clear(); lcd.print("      FIM!      ");
            xSemaphoreGive(mutexI2C);
          }
        }
      }
    }
    
    if (xSemaphoreTake(mutexI2C, portMAX_DELAY) == pdTRUE) {
      if (showModeScreen) {
        if (millis() >= modeScreenUntil) { 
            showModeScreen = false; lcd.clear(); 
        }
      } 
      else if (timerRunning) {
        int m = timerSecondsRemaining / 60;
        int s = timerSecondsRemaining % 60;
        printBigDigit(m / 10, 0);
        printBigDigit(m % 10, 4);
        lcd.setCursor(7, 0); lcd.write(6); 
        lcd.setCursor(7, 1); lcd.write(7); 
        printBigDigit(s / 10, 8);
        printBigDigit(s % 10, 12);
      } 
      else {
        lcd.setCursor(0, 0); lcd.print("Focus Cube     ");
        lcd.setCursor(0, 1); lcd.print("Modo: PARADO   ");
      }
      xSemaphoreGive(mutexI2C);
    }
    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}

void vTask_MQTT(void *pvParameters) {
  Face faceParaPublicar;
  for(;;) {
    if (WiFi.status() != WL_CONNECTED) { }
    if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) reconnectMQTT(); 
    if (WiFi.status() == WL_CONNECTED) mqttClient.loop();
    
    if (xQueueReceive(xMQTTQueue, &faceParaPublicar, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (mqttClient.connected()) {
         ModeConfig mode = getModeForFace(faceParaPublicar);
         mqttClient.publish(MQTT_TOPIC_STATUS, mode.name);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void setupWiFi() {
  lcd.clear(); lcd.setCursor(0,0); lcd.print("Conectando WiFi");
  Serial.print("Conectando WiFi: "); Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) { 
    delay(500); 
    Serial.print(".");
    lcd.setCursor(attempts % 16, 1); lcd.print(".");
    attempts++;
  }
  lcd.clear();
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi OK!");
    lcd.setCursor(0,0); lcd.print("WiFi Conectado!");
    delay(1000);
  } else {
    Serial.println("\nWiFi Falhou.");
    lcd.setCursor(0,0); lcd.print("WiFi Falhou!");
    delay(2000);
  }
}

void reconnectMQTT() {
  if (!mqttClient.connected()) {
      String clientId = "focuscube-" + String(random(0xffff), HEX);
      if (mqttClient.connect(clientId.c_str())) {
        mqttClient.subscribe(MQTT_TOPIC_COMMAND);
      }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char message[length + 1]; memcpy(message, payload, length); message[length] = '\0';           
  DisplayMessage msg; msg.type = MSG_TYPE_COMMAND; 
  strncpy(msg.data.command, message, 16); msg.data.command[16] = '\0'; 
  xQueueSend(xDisplayQueue, &msg, pdMS_TO_TICKS(10));
}

Face classifyFace(float ax, float ay, float az) {
  const float G = 9.80665f; const float T = 0.40f; 
  float gx = ax/G, gy = ay/G, gz = az/G;
  float abx = fabs(gx), aby = fabs(gy), abz = fabs(gz);
  if (abx >= aby && abx >= abz) { if (abx >= T) return gx > 0 ? FACE_POS_X : FACE_NEG_X; }
  else if (aby >= abx && aby >= abz) { if (aby >= T) return gy > 0 ? FACE_POS_Y : FACE_NEG_Y; }
  else { if (abz >= T) return gz > 0 ? FACE_POS_Z : FACE_NEG_Z; }
  return FACE_UNKNOWN;
}

ModeConfig getModeForFace(Face f) {
  switch (f) {
    case FACE_POS_Y: return {"FOCO 25", 25 * 60, TYPE_FOCUS};  
    case FACE_NEG_Y: return {"FOCO 30", 30 * 60, TYPE_FOCUS};  
    case FACE_POS_X: return {"FOCO 45", 45 * 60, TYPE_FOCUS};  
    case FACE_NEG_X: return {"FOCO 60", 60 * 60, TYPE_FOCUS};  
    
    case FACE_NEG_Z: return {"DESCANSAR", 0, TYPE_SMART_BREAK}; 
    case FACE_POS_Z: return {"PARADO", 0, TYPE_IDLE}; 
    
    default:         return {"PARADO", 0, TYPE_IDLE};
  }
}

byte custom_chars[8][8] = {
  { 31, 0, 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0, 0, 31 }, { 31, 0, 0, 0, 0, 0, 0, 31 },
  { 31, 0, 0, 0, 0, 0, 0, 31 }, { 24, 24, 24, 24, 24, 24, 24, 24 }, { 3, 3, 3, 3, 3, 3, 3, 3 },
  { 14, 14, 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0, 14, 14 }
};
void loadCustomChars() { for(int i=0; i<8; i++) lcd.createChar(i, custom_chars[i]); }

void printBigDigit(int digit, int col) {
  byte map[10][6] = {
    {255, 0, 255, 255, 1, 255}, {0, 5, 255, 1, 255, 1}, {0, 2, 255, 255, 1, 1},
    {0, 2, 255, 1, 2, 255}, {255, 1, 255, 32, 32, 255}, {255, 2, 0, 1, 2, 255},
    {255, 2, 0, 255, 2, 255}, {0, 0, 255, 32, 32, 255}, {255, 2, 255, 255, 2, 255},
    {255, 2, 255, 1, 2, 255}
  };
  lcd.setCursor(col, 0); lcd.write(map[digit][0]); lcd.write(map[digit][1]); lcd.write(map[digit][2]);
  lcd.setCursor(col, 1); lcd.write(map[digit][3]); lcd.write(map[digit][4]); lcd.write(map[digit][5]);
}