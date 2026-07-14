#ifndef FLASH_LOGGER_H
#define FLASH_LOGGER_H

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_SPIFlash.h>
#include <SdFat.h>

class FlashLogger {
public:
  FlashLogger(uint8_t csPin, SPIClass *spi = &SPI)
    : transport(csPin, spi), flash(&transport) {}

  bool begin(const char *filename) {
    logFilename = filename;

    SPI.setSCLK(PA5);
    SPI.setMISO(PA6);
    SPI.setMOSI(PA7);
    SPI.begin();

    if (!flash.begin()) {
      statusText = "FLASH CHIP NOT FOUND";
      return false;
    }

    if (!fatfs.begin(&flash)) {
      statusText = "FLASH NOT FAT-FORMATTED";
      return false;
    }

    if (!openFile()) {
      statusText = "COULD NOT OPEN LOG FILE";
      return false;
    }

    transmitPosition = logFile.size();
    statusText = "FLASH READY";
    return true;
  }

  bool ready() const {
    return fileOpen;
  }

  const char *status() const {
    return statusText;
  }

  void logHeaderIfEmpty() {
    if (!fileOpen) return;

    if (logFile.size() == 0) {
      logFile.println(
        "millis,"
        "accel_x_mps2,accel_y_mps2,accel_z_mps2,"
        "gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,"
        "temperature_c"
      );
      logFile.flush();
      transmitPosition = 0;
    }
  }

  void logIMU(
    uint32_t timestampMs,
    float accelX,
    float accelY,
    float accelZ,
    float gyroX,
    float gyroY,
    float gyroZ,
    float temperatureC
  ) {
    if (!fileOpen) return;

    logFile.print(timestampMs);
    logFile.print(",");
    logFile.print(accelX, 6);
    logFile.print(",");
    logFile.print(accelY, 6);
    logFile.print(",");
    logFile.print(accelZ, 6);
    logFile.print(",");
    logFile.print(gyroX, 6);
    logFile.print(",");
    logFile.print(gyroY, 6);
    logFile.print(",");
    logFile.print(gyroZ, 6);
    logFile.print(",");
    logFile.println(temperatureC, 6);
  }

  void flushIfDue(uint32_t nowMs, uint32_t intervalMs = 1000) {
    if (!fileOpen) return;

    if (nowMs - lastFlushMs >= intervalMs) {
      logFile.flush();
      lastFlushMs = nowMs;
    }
  }

  bool dumpNewData(Stream &out) {
    if (!reopenForRead()) return false;

    uint32_t fileSize = readFile.size();
    if (transmitPosition > fileSize) {
      transmitPosition = 0;
    }

    if (transmitPosition == fileSize) {
      readFile.close();
      openFile();
      return true;
    }

    readFile.seek(transmitPosition);
    out.println();
    out.println("===== BEGIN NEW FLASH IMU DATA =====");

    char buffer[64];
    while (readFile.available()) {
      int bytesRead = readFile.read(buffer, sizeof(buffer));
      if (bytesRead > 0) {
        out.write((const uint8_t *)buffer, bytesRead);
      }
    }

    transmitPosition = readFile.position();
    readFile.close();

    out.println();
    out.println("===== END NEW FLASH IMU DATA =====");
    out.println();

    openFile();
    return true;
  }

  bool dumpAllData(Stream &out) {
    transmitPosition = 0;
    return dumpNewData(out);
  }

  void close() {
    if (fileOpen) {
      logFile.flush();
      logFile.close();
      fileOpen = false;
    }
  }

private:
  bool openFile() {
    logFile = fatfs.open(logFilename, O_RDWR | O_CREAT | O_AT_END);
    fileOpen = (bool)logFile;
    if (fileOpen) {
      lastFlushMs = millis();
    }
    return fileOpen;
  }

  bool reopenForRead() {
    if (fileOpen) {
      logFile.flush();
      logFile.close();
      fileOpen = false;
    }

    readFile = fatfs.open(logFilename, O_READ);
    if (!readFile) {
      statusText = "COULD NOT OPEN LOG FILE FOR READ";
      return false;
    }

    return true;
  }

  const char *logFilename = "imu.csv";
  const char *statusText = "FLASH NOT STARTED";

  Adafruit_FlashTransport_SPI transport;
  Adafruit_SPIFlash flash;
  FatFileSystem fatfs;
  File32 logFile;
  File32 readFile;

  bool fileOpen = false;
  uint32_t lastFlushMs = 0;
  uint32_t transmitPosition = 0;
};

#endif
