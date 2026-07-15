#include "Buzzer.h"
#include "FlashLogger.h"
#include "IMU.h"

IMU imu;
FlashLogger flashLogger(PA4);


const int BUZZER_PIN = A1;
const unsigned long LOG_INTERVAL_MS = 10;
const unsigned long PRINT_INTERVAL_MS = 500;
const unsigned long SERIAL_WAIT_MS = 3000;
const unsigned long START_WAIT_MS = 5000;
unsigned long lastLogMs = 0;
unsigned long lastPrintMs = 0;

HardwareSerial BT(PA10, PA9);  // RX, TX

void printBoth(const char *message) {
  Serial.println(message);
  BT.println(message);
}

void setup() {
  Buzzer::startupTone(BUZZER_PIN);  //buzzer makes noise at start

  Serial.begin(115200);  // USB Serial Monitor
  BT.begin(9600);        // HC-06

  unsigned long serialStartMs = millis();
  while (!Serial && millis() - serialStartMs < SERIAL_WAIT_MS) {
    delay(10);
  }

  printBoth("Bluetooth Ready");

  waitForStart();

  if (!imu.begin()) {
    printBoth("IMU not found!");

    while (1)
      ;
  }

  printBoth("IMU initialized.");

  if (!flashLogger.begin("imu.csv")) {
    Serial.print("Flash logger failed: ");
    Serial.println(flashLogger.status());
    BT.print("Flash logger failed: ");
    BT.println(flashLogger.status());
    Buzzer::errorTone(BUZZER_PIN);
  } else {
    flashLogger.logHeaderIfEmpty();
    printBoth("Flash logger initialized.");
  }
}

void loop() {
  unsigned long now = millis();

  if (now - lastLogMs >= LOG_INTERVAL_MS) {
    IMUData data = imu.read();

    flashLogger.logIMU(
      data.timestamp,
      data.accelX,
      data.accelY,
      data.accelZ,
      data.gyroX,
      data.gyroY,
      data.gyroZ,
      data.temperature
    );
    flashLogger.flushIfDue(now);
    lastLogMs += LOG_INTERVAL_MS;

    if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
      Serial.print("Accel: ");
      Serial.print(data.accelX);
      Serial.print(", ");
      Serial.print(data.accelY);
      Serial.print(", ");
      Serial.println(data.accelZ);

      Serial.print("Gyro: ");
      Serial.print(data.gyroX);
      Serial.print(", ");
      Serial.print(data.gyroY);
      Serial.print(", ");
      Serial.println(data.gyroZ);

      Serial.print("Temp: ");
      Serial.println(data.temperature);

      lastPrintMs = now;
    }
  }

  delay(1);
}


void waitForStart() {
  String input = "";
  unsigned long startWaitBeganMs = millis();

  printBoth("Type 'start' to begin, or wait 5 seconds for auto-start.");

  while (true) {
    if (millis() - startWaitBeganMs >= START_WAIT_MS) {
      printBoth("Auto-starting...");
      return;
    }

    while (Serial.available()) {
      char c = Serial.read();

      if (c == '\n' || c == '\r') {
        input.trim();

        if (input.equalsIgnoreCase("start")) {
          printBoth("Starting...");
          return;
        }

        printBoth("Invalid command. Type 'start' to begin.");
        input = "";
      } else {
        input += c;
      }
    }
  }
}
