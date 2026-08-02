#include <Arduino.h>
#include <AccelStepper.h>
#include <HardwareSerial.h>
#include <func.h>
#include <vector>

// CRSF receive stuff
#define RXD1_PIN 20
#define TXD1_PIN 21
#define CRSF_BAUD_RATE 420000
#define CRSF_SYNC_BYTE 0xC8
#define RC_CHANNELS_PACKED 0x16

uint8_t AXE_CHANNEL = 9; // Канал пульта (Ch10)

// Axe C Motor pinout 
const int Axe_C_Step = 5;  
const int Axe_C_Dir  = 6;  
const int pinEn      = 4;  

// Буфер для зчитування байтів CRSF
std::vector<uint8_t> crsfBuffer;

// Глобальний масив для розпакованих каналів (16 каналів)
uint16_t channel[16] = {0};

// Прототипи функцій (мають бути реалізовані у вашому func.h)
uint8_t crc8_d5(const uint8_t *data, uint8_t len);
void unpackCh(const uint8_t *data, uint16_t *out);

// Ініціалізація об'єкта мотора Axe_C
AccelStepper stepper(AccelStepper::DRIVER, Axe_C_Step, Axe_C_Dir);

void setup() {
  Serial.begin(115200);
  delay(100);
  
  // Ініціалізація швидкісного порту для CRSF receiver
  Serial1.begin(CRSF_BAUD_RATE, SERIAL_8N1, RXD1_PIN, TXD1_PIN);
  delay(100);

  // Активація драйвера мотора
  pinMode(pinEn, OUTPUT);
  digitalWrite(pinEn, LOW); 
  
  // Конфігурація лімітів швидкості
  stepper.setMaxSpeed(4000.0); // Обмежуємо планку максимальної швидкості
  stepper.setSpeed(0);         // Початкова швидкість — мотор стоїть
}

void loop() {
  
  // 1. Швидке вичитування байтів із апаратного буфера в робочий вектор
  while (Serial1.available()) {
    crsfBuffer.push_back(Serial1.read());
  }

  // 2. Обробка пакета CRSF
  if (crsfBuffer.size() >= 2) {
    auto it = crsfBuffer.begin();
    while (it != crsfBuffer.end()) {
      if (*it == CRSF_SYNC_BYTE) {
        uint8_t len = *(it + 1);

        // Перевіряємо, чи пакет зайшов повністю
        if (crsfBuffer.size() >= (std::distance(crsfBuffer.begin(), it) + len + 2)) {
          const uint8_t* packetStart = &(*it);
          uint8_t received_crc = packetStart[len + 1];
          uint8_t computed_crc = crc8_d5(packetStart + 2, len - 1);

          //  Пряме порівняння байту типу пакету (packetStart[2])
          if ((computed_crc == received_crc) && (packetStart[2] == RC_CHANNELS_PACKED)) {
            unpackCh(packetStart + 3, channel); 
           
            // Отримуємо поточне значення з пульта (зазвичай діапазон від 172 до 1811, де ~992 середина)
            uint16_t channelValue = channel[AXE_CHANNEL];
                        
            // --- ДИНАМІЧНА ЗМІНА ШВИДКОСТІ ---
            // Створюємо мертву зону (Neutral Zone) в центрі джойстика, щоб мотор не сіпався від шумів
            if (channelValue >= 950 && channelValue <= 1030) {
              stepper.setSpeed(0); 
            } 
            else if (channelValue < 950) {
              // Джойстик відхилено назад/ліворуч. 
              // Пропорційно розраховуємо швидкість від 0 до -2000 кроків/сек
              float speedFactor = (950.0 - channelValue) / (950.0 - 172.0);
              stepper.setSpeed(-1 * (speedFactor * 2000.0)); 
            } 
            else if (channelValue > 1030) {
              // Джойстик відхилено вперед/праворуч.
              // Пропорційно розраховуємо швидкість від 0 до +2000 кроків/сек
              float speedFactor = (channelValue - 1030.0) / (1811.0 - 1030.0);
              stepper.setSpeed(speedFactor * 2000.0);
            }
          }
          it += len + 2; 
        } else {
          break; // Пакет не повний, чекаємо наступного loop()
        }
      } else {
        it++;
      }
    }
    // Очищуємо відпрацьовані дані з вектора
    crsfBuffer.erase(crsfBuffer.begin(), it);
  }

 /* // 3. Періодичний вивід реальної швидкості мотора в монітор порту для діагностики
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 100) { 
    Serial.printf("Ch%d Value: %4u | Real-Time Motor Speed: %.1f steps/s\n", 
                  AXE_CHANNEL + 1, channel[AXE_CHANNEL], stepper.speed());
    lastPrintTime = millis();
  }*/

  // --- ГОЛОВНИЙ ВИКОНАВЧИЙ ІМПУЛЬС ---
  // Працює на кожному циклі мікроконтролера. 
  // Розраховує інтервал динамічно на основі того значення, яке прийшло з умов вище.
  stepper.runSpeed();
}