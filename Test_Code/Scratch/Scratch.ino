#include "Buzzer.h"
#include "IMU.h"

IMU imu;


const int BUZZER_PIN = A1;

HardwareSerial BT(PA10, PA9);  // RX, TX

void setup() {
  Buzzer::startupTone(BUZZER_PIN);  //buzzer makes noise at start

  Serial.begin(115200);  // USB Serial Monitor
  BT.begin(9600);        // HC-06

  Serial.println("Bluetooth Ready");

  while (!Serial) {
    delay(10);
  }

  waitForStart();

  if (!imu.begin()) {
    Serial.println("IMU not found!");

    while (1)
      ;
  }

  Serial.println("IMU initialized.");
}

void loop() {
  IMUData data = imu.read();

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

  delay(500);
}


void waitForStart() {
  String input = "";

  Serial.println("Type 'start' to begin.");

  while (true) {
    while (Serial.available()) {
      char c = Serial.read();

      if (c == '\n' || c == '\r') {
        input.trim();

        if (input.equalsIgnoreCase("start")) {
          Serial.println("Starting...");
          return;
        }

        Serial.println("Invalid command. Type 'start' to begin.");
        input = "";
      } else {
        input += c;
      }
    }
  }
}
