#include <Arduino.h>
#include <AccelStepper.h>
#include <HardwareSerial.h>
#include <func.h>
#include <vector>

// CRSF receive stuff
#define RXD1_PIN 20
#define TXD1_PIN 21
#define CRSF_BAUD_RATE 420000
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_ADDRESS_TRANSMITTER 0xEE
#define RC_CHANNELS_PACKED 0x16

// uint8_t AXE_C_CHANNEL = 14; // Канал пульта (Ch15)
uint8_t AXE_A_CHANNEL = 15; // Канал пульта (Ch16)

// Axe C Motor pinout 
const int pinEn      = 3; // Common enable pin

const int Axe_C_Step = 4;  
const int Axe_C_Dir  = 5; 

const int Axe_A_Step = 6;  
const int Axe_A_Dir  = 7; 
  

// Буфер для зчитування байтів CRSF
std::vector<uint8_t> crsfBuffer;

// Глобальний масив для розпакованих каналів (16 каналів)
uint16_t channel[16] = {0};

// Прототипи функцій (мають бути реалізовані у вашому func.h)
uint8_t crc8_d5(const uint8_t *data, uint8_t len);
void unpackCh(const uint8_t *data, uint16_t *out);

// initilize motors
AccelStepper stepperC(AccelStepper::DRIVER, Axe_C_Step, Axe_C_Dir);
AccelStepper stepperA(AccelStepper::DRIVER, Axe_A_Step, Axe_A_Dir);

void setup() {
  Serial.begin(115200);
  delay(100);
  
  // Ініціалізація швидкісного порту для CRSF receiver
  Serial1.begin(CRSF_BAUD_RATE, SERIAL_8N1, RXD1_PIN, TXD1_PIN, /*inverted=*/ true);
  delay(100);

  // Активація драйверів моторів (спільна лінія EN)
  pinMode(pinEn, OUTPUT);
  digitalWrite(pinEn, LOW); 
  
  // Конфігурація лімітів швидкості для Motor C
  stepperC.setMaxSpeed(4000.0);
  stepperC.setSpeed(0);        

  // Конфігурація лімітів швидкості для Motor A
  stepperA.setMaxSpeed(4000.0);
  stepperA.setSpeed(0);
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
      if (*it == CRSF_ADDRESS_TRANSMITTER) {
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
            //uint16_t Channel_C_Value = channel[AXE_C_CHANNEL];
            uint16_t Channel_A_Value = channel[AXE_A_CHANNEL];
/////////////////////////////////////////////////////////////////////////////////////////////////////////////                      
            // --- КЕРУВАННЯ МОТОРОМ C ---
            if (Channel_A_Value >= 950 && Channel_A_Value <= 1030) {
              stepperC.setSpeed(0); 
            } 
            else if (Channel_A_Value >= 727 && Channel_A_Value <= 767) {
               stepperC.setSpeed(500); 
            } 
            else if (Channel_A_Value >= 563 && Channel_A_Value <= 603) {
              stepperC.setSpeed(-500);
            }
            else{
              stepperC.setSpeed(0);
            }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////  
            // --- КЕРУВАННЯ МОТОРОМ A ---
            if (Channel_A_Value >= 950 && Channel_A_Value <= 1030) {
              stepperA.setSpeed(0); 
            } 
            else if (Channel_A_Value >= 235 && Channel_A_Value <= 275) {
              stepperA.setSpeed(500); 
            } 
            else if (Channel_A_Value >= 399 && Channel_A_Value <= 439) {
              stepperA.setSpeed(-500);
            }
            else{
              stepperA.setSpeed(0);
            }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////  
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
       /*
  // 3. Діагностика у монітор порту
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 100) { 
    Serial.printf("Ch%d Value: %4u | Motor C Speed: %.1f steps/s Ch%d Value: %4u | Motor A Speed: %.1f steps/s\n", 
                  AXE_A_CHANNEL + 1, channel[AXE_A_CHANNEL], stepperC.speed(),
                  AXE_A_CHANNEL + 1, channel[AXE_A_CHANNEL], stepperA.speed());
    lastPrintTime = millis();
  }   */

  // --- ГОЛОВНИЙ ВИКОНАВЧИЙ ІМПУЛЬС ---
  // Працює на кожному циклі мікроконтролера. 
  // Розраховує інтервал динамічно на основі того значення, яке прийшло з умов вище.
  stepperC.runSpeed();
  stepperA.runSpeed();
}