#include <Arduino.h>
#include <AccelStepper.h>
#include <HardwareSerial.h>
#include <func.h>
#include <vector>

#define LED 8   // LED_BUILTIN

// CRSF reccive stuff
#define RXD1_PIN 20
#define TXD1_PIN 21
#define CRSF_BAUD_RATE 420000
#define CRSF_SYNC_BYTE 0xC8
#define RC_CHANNELS_PACKED 0x16
// Channel stuff
uint8_t AXE_CHANNEL = 9;

// Axe C Motor pinout 
const int Axe_C_Step = 5;  // CLK / STP / Horisontal Axe
const int Axe_C_Dir  = 6;  // DIR

// Axe A Motor pinout
const int Axe_A_Step = 7;  // CLK / STP / Vertical Axe
const int Axe_A_Dir  = 8;  // DIR

const int pinEn   = 4;  // EN (Enable)

uint8_t speed = 0;

////////////////////////////////////////////////////////// CRSF VECTOR AJUST  /////////////////////////////////////////////////

// A vector to hold incoming CRSF bytes
std::vector<uint8_t> crsfBuffer;

// Globals to store the parsed RC channel data
uint16_t channel[16] = {0};

// --- Function Prototypes ---
uint8_t crc8_d5(const uint8_t *data, uint8_t len);
void unpackCh(const uint8_t *data, uint16_t *out);

//////////////////////////////////////////////////////// STEPPER MOTOR DRIVER AJUST /////////////////////////////////////////

// Розрахунок: 200 кроків * 1 мікрокрок * 51 редуктор = 10200
const long stepsPerRevolution = 10200; 
const uint8_t stepsPer3Degre = 85;

// Ініціалізація бібліотеки (тип драйвера 1 — зі змінними Step/Dir)
AccelStepper stepper(AccelStepper::DRIVER, Axe_C_Step, Axe_C_Dir);


void setup() {

  ////////////////////////////////////////////////////  UART AJUST  ////////////////////////////////////////////////////////
  Serial.begin(115200);
  delay(100);
  Serial1.begin(CRSF_BAUD_RATE, SERIAL_8N1, RXD1_PIN, TXD1_PIN);
   delay(100);

  // 1. Initialize LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  ////////////////////////////////////////////////// STEPPER MOTORS AJUST /////////////////////////////////////////////
  pinMode(pinEn, OUTPUT);
  digitalWrite(pinEn, LOW); // Активація драйвера SERVO42C
  
  //////////////////////////////////////////////////   SPEED & ACELERATION AJUST  /////////////////////////////////////
  stepper.setMaxSpeed(5000.0);     // 1000 Максимальна швидкість (кроків/сек)
  stepper.setAcceleration(500.0);  // 400 Плавне прискорення (дуже важливо для тяги!)
}

void loop() {
  /////////////////////////////////////////////////////////// CRSF PARSING ///////////////////////////////////////////////////
 
  while (Serial1.available()) {
    // Read all available bytes into the buffer
    crsfBuffer.push_back(Serial1.read());
  }

  // If there's enough data to potentially contain a packet
  if (crsfBuffer.size() >= 2) {
    auto it = crsfBuffer.begin();
    while (it != crsfBuffer.end()) {
      if (*it == (CRSF_SYNC_BYTE || 0xEE)) {
        // Found the sync byte. Get the reported length.
        uint8_t len = *(it + 1);

        // Check if we have received the full packet.
        if (crsfBuffer.size() >= (std::distance(crsfBuffer.begin(), it) + len + 2)) {
          // A full packet is available.
          const uint8_t* packetStart = &(*it);
          uint8_t received_crc = packetStart[len + 1];
          uint8_t computed_crc = crc8_d5(packetStart + 2, len - 1);

          // Check if CRC is correct and packet is a channel packet
          if ((computed_crc == received_crc) && (packetStart[2] == RC_CHANNELS_PACKED)) {
            unpackCh(packetStart + 3, channel); // +3 to skip Sync, Length, and Type bytes
           
            
                        // --- START OF RANGE LOGIC ---
                        
                        uint16_t channelValue = channel[AXE_CHANNEL];
                        
                        // Using direct, hardcoded checks for the boundaries
                        if (channelValue < 461) { // Range 1: 191 - 461
                                     speed *= 1;
                        } else if (channelValue < 731) { // Range 2: 461 – 731
                                     speed *= 2; 
                        } else if (channelValue < 797) {
                                     speed *= 0; // Range 3: 731 – 1001
                        } else if (channelValue < 999) {
                                     speed *= 0; // Range 4: 1001 – 1271
                        } else if (channelValue < 1605) {
                                     speed *= -1; // Range 7: 1271 – 1541
                        } else { 
                                     speed *= -2; // Range 8: 1541 - 1811 (and anything higher)
                        }


                        



            // --- PRINT CHANNELS HERE ---
            // The RC_CHANNELS_PACKED packet contains 16 channels.
            Serial.printf("Channels:");
            for (int i = 0; i < 16; i++) {
              // Print each channel with a label and value
              Serial.printf(" Ch%d:%4u", i + 1, channel[i]);
            }
            Serial.println(); // Newline at the end
                        
          }
          
          // Move past the current packet
          it += len + 2; 
        } else {
          // Incomplete packet. Stop parsing this time.
          break;
        }
      } else {
        // Not a sync byte, advance to the next byte
        it++;
      }
    }
    
    // Erase processed data from the buffer
    crsfBuffer.erase(crsfBuffer.begin(), it);
  }

  // Example: print the channel data to the console periodically
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 100) { // Print every 100ms
    Serial.printf("\rCH1:%4u CH2:%4u CH3:%4u CH4:%4u", channel[0], channel[1], channel[2], channel[3]);
    lastPrintTime = millis();
  }


  ////////////////////////////////////////////////////////////  TEST ROTATION  //////////////////////////////////////////////

  // Обертання на 1 повний оберт редуктора вперед
  stepper.moveTo(stepsPerRevolution);
  stepper.runToPosition(); // Блокуючий рух із урахуванням розгону та гальмування
  

  // Повернення у вихідну позицію (0)
  // stepper.moveTo(0);
  // stepper.runToPosition();
  
}

