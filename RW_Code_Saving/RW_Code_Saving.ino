#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <Adafruit_SPIFlash.h>
#include <SdFat.h>
#include <math.h>

// =====================================================
// PIN SETUP BASED ON YOUR WIRING
// =====================================================

// Bluetooth UART
// Constructor format: RX, TX
// PA10 = MCU RX from Bluetooth TX
// PA9  = MCU TX to Bluetooth RX
HardwareSerial BTSerial(PA10, PA9);

// IMU I2C
// PB6 = SCL IMU
// PB7 = SDA IMU

// SPI bus for W25Q64JV flash
// PA5 = SCK
// PA6 = MISO
// PA7 = MOSI
// PA4 = CS Flash

const int FLASH_CS_PIN = PA4;
const int ESC_PIN      = PB10;
const int LED_PIN      = PB13;

// =====================================================
// OBJECTS
// =====================================================

Adafruit_LSM6DSO32 dso32;
Servo esc;

// W25Q64JV SPI flash storage
Adafruit_FlashTransport_SPI flashTransport(FLASH_CS_PIN, &SPI);
Adafruit_SPIFlash flash(&flashTransport);
FatFileSystem fatfs;
File32 telemetryFile;

// =====================================================
// ESC SETTINGS
// =====================================================

const int ESC_REVERSE_MAX = 1000;
const int ESC_NEUTRAL     = 1500;
const int ESC_FORWARD_MAX = 2000;

int escPulseWidth = ESC_NEUTRAL;

// =====================================================
// FLASH STORAGE SETTINGS
// =====================================================

enum FlashStatus {
  FLASH_NOT_CHECKED,
  FLASH_NOT_FOUND,
  FLASH_UNFORMATTED,
  FLASH_READY
};

FlashStatus flashStatus = FLASH_NOT_CHECKED;

const char* LOG_FILE_NAME = "imu.csv";

// Save IMU data to flash every 200 ms.
// 200 ms = 5 Hz logging.
const unsigned long LOG_INTERVAL_MS = 200;

// Transmit saved flash data over Bluetooth every 30 seconds.
const unsigned long TRANSMIT_INTERVAL_MS = 30000;

// Flush flash file every 1 second.
const unsigned long FLUSH_INTERVAL_MS = 1000;

// If flash is missing or unformatted, remind over Serial/Bluetooth every 10 seconds.
const unsigned long FLASH_ERROR_PRINT_INTERVAL_MS = 10000;

unsigned long lastLogTime = 0;
unsigned long lastTransmitTime = 0;
unsigned long lastFlushTime = 0;
unsigned long lastFlashErrorPrintTime = 0;

// Tracks where the last Bluetooth transmission stopped.
// This prevents resending the same flash data every 30 seconds.
uint32_t lastTransmitPosition = 0;

// =====================================================
// FLIGHT STATE
// =====================================================

enum { prelaunch, launch, coast, descend } state = prelaunch;

// =====================================================
// KALMAN FILTER STRUCT
// =====================================================

typedef struct {
  float Q_angle;
  float Q_bias;
  float R_measure;
  float angle;
  float bias;
  float rate;
  float P[2][2];
} Kalman_t;

Kalman_t KalmanX;
Kalman_t KalmanY;

uint32_t timer;

// =====================================================
// SENSOR / CONTROL VARIABLES
// =====================================================

float gyroX_offset = 0;
float gyroY_offset = 0;
float gyroZ_offset = 0;

int num_samples = 500;

float current_error = 0;
float previous_error = 0;

float prop_error = 0;
float total_integrated_error = 0;
float integral_error = 0;
float derivative_error = 0;
float motor_control = 0;

float Kp = -2.5873;
float Ki = -5.9426;
float Kd = 0;

const float MAX_PID_OUTPUT = 12.0f;

float launch_thresh = 9.81f * 1.5f;

// =====================================================
// FUNCTION PROTOTYPES
// =====================================================

float PIDController(float error, float prev_error, double dt);

void LSM6DSO32Setup();
void calibrategyro();

void Kalman_Init(Kalman_t *kf);
float Kalman_GetAngle(Kalman_t *kf, float newAngle, float newRate, float dt);

void armESC();
void sendESCPWM(float control_signal);

const char* stateName(int s);

void setupFlashStorage();
bool openTelemetryFileForAppend();

void logIMUToFlash(
  sensors_event_t accel,
  sensors_event_t gyro,
  sensors_event_t temp,
  float accel_mag,
  float gyroRateX,
  float gyroRateY,
  float gyroRateZ,
  float roll,
  float pitch,
  double dt
);

void transmitFlashLogOverBluetooth();
void printFlashProblemMessage();

// =====================================================
// SETUP
// =====================================================

void setup(void) {
  // 1. ESC arming
  esc.attach(ESC_PIN, ESC_REVERSE_MAX, ESC_FORWARD_MAX);
  armESC();

  // 2. Serial initialization
  Serial.begin(115200);
  BTSerial.begin(9600);

  uint32_t timeout = millis();
  while (!Serial && (millis() - timeout < 3000)) {}

  Serial.println("System Booting...");
  BTSerial.println("System Booting...");

  // 3. Flash storage check
  setupFlashStorage();

  // 4. IMU initialization
  LSM6DSO32Setup();

  Serial.println("Calibrating Gyro...");
  BTSerial.println("Calibrating Gyro...");

  calibrategyro();

  Serial.println("Calibration Complete.");
  BTSerial.println("Calibration Complete.");

  // 5. Kalman filters
  Kalman_Init(&KalmanX);
  Kalman_Init(&KalmanY);

  timer = micros();

  pinMode(LED_PIN, OUTPUT);
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  sensors_event_t accel, gyro, temp;

  static unsigned long last_time = 0;

  double dt = (double)(micros() - timer) / 1000000.0;
  timer = micros();

  dso32.getEvent(&accel, &gyro, &temp);

  float gyroRateX = (gyro.gyro.x - gyroX_offset) * 57.29578f;
  float gyroRateY = (gyro.gyro.y - gyroY_offset) * 57.29578f;
  float gyroRateZ = (gyro.gyro.z - gyroZ_offset) * 57.29578f;

  float accel_mag = sqrt(
    pow(accel.acceleration.x, 2) +
    pow(accel.acceleration.y, 2) +
    pow(accel.acceleration.z, 2)
  );

  float accRoll = atan2(
    accel.acceleration.y,
    accel.acceleration.z
  ) * 57.29578f;

  float accPitch = atan2(
    -accel.acceleration.x,
    sqrt(
      pow(accel.acceleration.y, 2) +
      pow(accel.acceleration.z, 2)
    )
  ) * 57.29578f;

  float roll = Kalman_GetAngle(&KalmanX, accRoll, gyroRateX, dt);
  float pitch = Kalman_GetAngle(&KalmanY, accPitch, gyroRateY, dt);

  // Yaw-rate error for reaction wheel PID.
  // Adafruit gyro values are in rad/s.
  current_error = gyro.gyro.z - gyroZ_offset;

  switch (state) {
    case prelaunch:
      if (millis() - last_time > 1000) {
        Serial.println("STATE: PRELAUNCH");

        BTSerial.print("Acc[X,Y,Z]: [");
        BTSerial.print(accel.acceleration.x);
        BTSerial.print(", ");
        BTSerial.print(accel.acceleration.y);
        BTSerial.print(", ");
        BTSerial.print(accel.acceleration.z);
        BTSerial.print("]\t");

        BTSerial.print("Mag: ");
        BTSerial.print(accel_mag);
        BTSerial.print("\t");

        BTSerial.print("Temp: ");
        BTSerial.print(temp.temperature);
        BTSerial.print(" C\t");

        BTSerial.print("Roll: ");
        BTSerial.print(roll);
        BTSerial.print("\t");

        BTSerial.print("Pitch: ");
        BTSerial.println(pitch);

        last_time = millis();
      }

      if (fabs(accel_mag - 9.81f) > 2.0f) {
        BTSerial.print("[FLAG: BAD_PAD_CALIBRATION] ");
      }

      if (accel_mag >= launch_thresh) {
        state = launch;

        Serial.println("LAUNCH DETECTED!");
        BTSerial.println("LAUNCH DETECTED!");
      }
      break;

    case launch:
      // Add better transition logic later if needed.
      state = coast;
      break;

    case coast:
      motor_control = PIDController(current_error, previous_error, dt);

      if (motor_control < 0) {
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
      }

      sendESCPWM(motor_control);
      break;

    case descend:
      esc.writeMicroseconds(ESC_NEUTRAL);
      escPulseWidth = ESC_NEUTRAL;
      break;
  }

  // Save IMU data to flash only if flash is detected and FAT-formatted.
  if (millis() - lastLogTime >= LOG_INTERVAL_MS) {
    logIMUToFlash(
      accel,
      gyro,
      temp,
      accel_mag,
      gyroRateX,
      gyroRateY,
      gyroRateZ,
      roll,
      pitch,
      dt
    );

    lastLogTime = millis();
  }

  // Every 30 seconds, transmit newly saved flash data over Bluetooth.
  if (millis() - lastTransmitTime >= TRANSMIT_INTERVAL_MS) {
    transmitFlashLogOverBluetooth();
    lastTransmitTime = millis();
  }

  // If flash is missing or unformatted, print a reminder periodically.
  if (flashStatus != FLASH_READY) {
    if (millis() - lastFlashErrorPrintTime >= FLASH_ERROR_PRINT_INTERVAL_MS) {
      printFlashProblemMessage();
      lastFlashErrorPrintTime = millis();
    }
  }

  previous_error = current_error;
}

// =====================================================
// ESC FUNCTIONS
// =====================================================

void armESC() {
  escPulseWidth = ESC_NEUTRAL;
  esc.writeMicroseconds(ESC_NEUTRAL);
  delay(1000);
}

void sendESCPWM(float control_signal) {
  if (fabs(control_signal) < 0.05f) {
    escPulseWidth = ESC_NEUTRAL;
    esc.writeMicroseconds(ESC_NEUTRAL);
    return;
  }

  float throttle_ratio = control_signal / MAX_PID_OUTPUT;

  int pulseWidth = ESC_NEUTRAL + (int)(throttle_ratio * 500);

  pulseWidth = constrain(pulseWidth, ESC_REVERSE_MAX, ESC_FORWARD_MAX);

  escPulseWidth = pulseWidth;

  esc.writeMicroseconds(pulseWidth);
}

// =====================================================
// IMU FUNCTIONS
// =====================================================

void LSM6DSO32Setup() {
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin();

  if (!dso32.begin_I2C()) {
    while (1) {
      Serial.println("I2C Not Found");
      BTSerial.println("I2C Not Found");
      delay(500);
    }
  }

  dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);
  dso32.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  dso32.setAccelDataRate(LSM6DS_RATE_833_HZ);
  dso32.setGyroDataRate(LSM6DS_RATE_833_HZ);
}

void calibrategyro() {
  float x_sum = 0;
  float y_sum = 0;
  float z_sum = 0;

  for (int i = 0; i < num_samples; i++) {
    sensors_event_t accel, gyro, temp;
    dso32.getEvent(&accel, &gyro, &temp);

    x_sum += gyro.gyro.x;
    y_sum += gyro.gyro.y;
    z_sum += gyro.gyro.z;

    delay(10);
  }

  gyroX_offset = x_sum / num_samples;
  gyroY_offset = y_sum / num_samples;
  gyroZ_offset = z_sum / num_samples;
}

// =====================================================
// PID FUNCTION
// =====================================================

float PIDController(float error, float prev_error, double dt) {
  if (dt <= 0.0) {
    return 0.0;
  }

  prop_error = Kp * error;

  total_integrated_error += error * dt;

  if (Ki != 0) {
    float max_accum = fabs(MAX_PID_OUTPUT / Ki);
    total_integrated_error = constrain(total_integrated_error, -max_accum, max_accum);
    integral_error = Ki * total_integrated_error;
  } else {
    integral_error = 0;
  }

  derivative_error = Kd * ((error - prev_error) / dt);

  float control_output = prop_error + integral_error + derivative_error;

  return constrain(control_output, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
}

// =====================================================
// KALMAN FILTER FUNCTIONS
// =====================================================

void Kalman_Init(Kalman_t *kf) {
  kf->Q_angle = 0.001f;
  kf->Q_bias = 0.003f;
  kf->R_measure = 0.03f;

  kf->angle = 0.0f;
  kf->bias = 0.0f;

  kf->P[0][0] = 0.0f;
  kf->P[0][1] = 0.0f;
  kf->P[1][0] = 0.0f;
  kf->P[1][1] = 0.0f;
}

float Kalman_GetAngle(Kalman_t *kf, float newAngle, float newRate, float dt) {
  kf->rate = newRate - kf->bias;
  kf->angle += dt * kf->rate;

  kf->P[0][0] += dt * (
    dt * kf->P[1][1] -
    kf->P[0][1] -
    kf->P[1][0] +
    kf->Q_angle
  );

  kf->P[0][1] -= dt * kf->P[1][1];
  kf->P[1][0] -= dt * kf->P[1][1];
  kf->P[1][1] += kf->Q_bias * dt;

  float S = kf->P[0][0] + kf->R_measure;

  float K[2];
  K[0] = kf->P[0][0] / S;
  K[1] = kf->P[1][0] / S;

  float y = newAngle - kf->angle;

  kf->angle += K[0] * y;
  kf->bias  += K[1] * y;

  float P00_temp = kf->P[0][0];
  float P01_temp = kf->P[0][1];

  kf->P[0][0] -= K[0] * P00_temp;
  kf->P[0][1] -= K[0] * P01_temp;
  kf->P[1][0] -= K[1] * P00_temp;
  kf->P[1][1] -= K[1] * P01_temp;

  return kf->angle;
}

// =====================================================
// FLASH STORAGE FUNCTIONS
// =====================================================

const char* stateName(int s) {
  switch (s) {
    case prelaunch:
      return "prelaunch";

    case launch:
      return "launch";

    case coast:
      return "coast";

    case descend:
      return "descend";

    default:
      return "unknown";
  }
}

void setupFlashStorage() {
  Serial.println("Initializing W25Q64JV SPI flash...");
  BTSerial.println("Initializing W25Q64JV SPI flash...");

  // Force SPI pins to match your wiring.
  SPI.setSCLK(PA5);
  SPI.setMISO(PA6);
  SPI.setMOSI(PA7);
  SPI.begin();

  // Step 1: Check if the physical flash chip responds.
  if (!flash.begin()) {
    flashStatus = FLASH_NOT_FOUND;

    Serial.println("FLASH CHIP FAILED OR NOT PRESENT");
    Serial.println("Check wiring: VCC=3.3V, GND, PA5 SCK, PA6 MISO, PA7 MOSI, PA4 CS.");

    BTSerial.println("FLASH CHIP FAILED OR NOT PRESENT");
    BTSerial.println("Check wiring: VCC=3.3V, GND, PA5 SCK, PA6 MISO, PA7 MOSI, PA4 CS.");

    return;
  }

  Serial.println("FLASH CHIP FOUND");
  BTSerial.println("FLASH CHIP FOUND");

  // Step 2: Check if a FAT filesystem exists.
  if (!fatfs.begin(&flash)) {
    flashStatus = FLASH_UNFORMATTED;

    Serial.println("FLASH CHIP FOUND, BUT FILESYSTEM FAILED");
    Serial.println("FLASH IS PROBABLY UNFORMATTED");
    Serial.println("Run Adafruit SPIFlash > fatfs_format first.");
    Serial.println("Then re-upload this reaction wheel code.");

    BTSerial.println("FLASH CHIP FOUND, BUT FILESYSTEM FAILED");
    BTSerial.println("FLASH IS PROBABLY UNFORMATTED");
    BTSerial.println("Run Adafruit SPIFlash > fatfs_format first.");
    BTSerial.println("Then re-upload this reaction wheel code.");

    return;
  }

  // Step 3: Filesystem works.
  flashStatus = FLASH_READY;

  Serial.println("FLASH FILESYSTEM READY");
  BTSerial.println("FLASH FILESYSTEM READY");

  if (!openTelemetryFileForAppend()) {
    flashStatus = FLASH_UNFORMATTED;

    Serial.println("Could not open imu.csv even though filesystem mounted.");
    Serial.println("Flash may need to be reformatted.");

    BTSerial.println("Could not open imu.csv even though filesystem mounted.");
    BTSerial.println("Flash may need to be reformatted.");

    return;
  }

  Serial.println("Flash logging ready.");
  BTSerial.println("Flash logging ready.");
}

bool openTelemetryFileForAppend() {
  if (flashStatus != FLASH_READY) {
    return false;
  }

  if (telemetryFile) {
    telemetryFile.close();
  }

  telemetryFile = fatfs.open(LOG_FILE_NAME, O_RDWR | O_CREAT | O_AT_END);

  if (!telemetryFile) {
    return false;
  }

  // If the file is empty, write the CSV header.
  if (telemetryFile.size() == 0) {
    telemetryFile.println(
      "millis,micros,state,"
      "accel_x_mps2,accel_y_mps2,accel_z_mps2,accel_mag_mps2,"
      "gyro_x_raw_rad_s,gyro_y_raw_rad_s,gyro_z_raw_rad_s,"
      "gyro_x_dps,gyro_y_dps,gyro_z_dps,"
      "temp_c,roll_deg,pitch_deg,"
      "esc_pwm_us,dt"
    );

    telemetryFile.flush();

    // First Bluetooth transmission should include the header.
    lastTransmitPosition = 0;
  } else {
    // Existing file from previous boot.
    // Start Bluetooth transmission from the current end of file,
    // so it only transmits new data from this run.
    lastTransmitPosition = telemetryFile.size();
  }

  return true;
}

void logIMUToFlash(
  sensors_event_t accel,
  sensors_event_t gyro,
  sensors_event_t temp,
  float accel_mag,
  float gyroRateX,
  float gyroRateY,
  float gyroRateZ,
  float roll,
  float pitch,
  double dt
) {
  if (flashStatus != FLASH_READY) {
    return;
  }

  if (!telemetryFile) {
    if (!openTelemetryFileForAppend()) {
      flashStatus = FLASH_UNFORMATTED;
      return;
    }
  }

  telemetryFile.print(millis());
  telemetryFile.print(",");

  telemetryFile.print(micros());
  telemetryFile.print(",");

  telemetryFile.print(stateName(state));
  telemetryFile.print(",");

  telemetryFile.print(accel.acceleration.x, 6);
  telemetryFile.print(",");

  telemetryFile.print(accel.acceleration.y, 6);
  telemetryFile.print(",");

  telemetryFile.print(accel.acceleration.z, 6);
  telemetryFile.print(",");

  telemetryFile.print(accel_mag, 6);
  telemetryFile.print(",");

  telemetryFile.print(gyro.gyro.x, 6);
  telemetryFile.print(",");

  telemetryFile.print(gyro.gyro.y, 6);
  telemetryFile.print(",");

  telemetryFile.print(gyro.gyro.z, 6);
  telemetryFile.print(",");

  telemetryFile.print(gyroRateX, 6);
  telemetryFile.print(",");

  telemetryFile.print(gyroRateY, 6);
  telemetryFile.print(",");

  telemetryFile.print(gyroRateZ, 6);
  telemetryFile.print(",");

  telemetryFile.print(temp.temperature, 6);
  telemetryFile.print(",");

  telemetryFile.print(roll, 6);
  telemetryFile.print(",");

  telemetryFile.print(pitch, 6);
  telemetryFile.print(",");

  telemetryFile.print(escPulseWidth);
  telemetryFile.print(",");

  telemetryFile.println(dt, 6);

  if (millis() - lastFlushTime >= FLUSH_INTERVAL_MS) {
    telemetryFile.flush();
    lastFlushTime = millis();
  }
}

void transmitFlashLogOverBluetooth() {
  if (flashStatus != FLASH_READY) {
    printFlashProblemMessage();
    return;
  }

  // Make sure latest data is saved before reading it back.
  if (telemetryFile) {
    telemetryFile.flush();
    telemetryFile.close();
  }

  File32 readFile = fatfs.open(LOG_FILE_NAME, O_READ);

  if (!readFile) {
    BTSerial.println("Could not open imu.csv for Bluetooth transmission.");

    Serial.println("Could not open imu.csv for Bluetooth transmission.");

    openTelemetryFileForAppend();
    return;
  }

  uint32_t fileSize = readFile.size();

  // If the file was erased/reset somehow, restart transmission from the beginning.
  if (lastTransmitPosition > fileSize) {
    lastTransmitPosition = 0;
  }

  // If no new data was added, do not spam Bluetooth.
  if (lastTransmitPosition == fileSize) {
    BTSerial.println();
    BTSerial.println("No new flash IMU data to transmit.");
    BTSerial.println();

    Serial.println("No new flash IMU data to transmit.");

    readFile.close();
    openTelemetryFileForAppend();
    return;
  }

  readFile.seek(lastTransmitPosition);

  BTSerial.println();
  BTSerial.println("===== BEGIN NEW FLASH IMU DATA =====");

  char buffer[64];

  while (readFile.available()) {
    int bytesRead = readFile.read(buffer, sizeof(buffer));

    if (bytesRead > 0) {
      BTSerial.write((uint8_t*)buffer, bytesRead);
    }
  }

  lastTransmitPosition = readFile.position();

  readFile.close();

  BTSerial.println();
  BTSerial.println("===== END NEW FLASH IMU DATA =====");
  BTSerial.println();

  Serial.println("New flash IMU data transmitted over Bluetooth.");

  // Reopen file for more logging after transmission.
  openTelemetryFileForAppend();
}

void printFlashProblemMessage() {
  if (flashStatus == FLASH_NOT_FOUND) {
    Serial.println("FLASH ERROR: W25Q64JV not detected.");
    Serial.println("Check 3.3V power, GND, PA5 SCK, PA6 MISO, PA7 MOSI, PA4 CS.");

    BTSerial.println("FLASH ERROR: W25Q64JV not detected.");
    BTSerial.println("Check 3.3V power, GND, PA5 SCK, PA6 MISO, PA7 MOSI, PA4 CS.");
  }

  else if (flashStatus == FLASH_UNFORMATTED) {
    Serial.println("FLASH ERROR: W25Q64JV detected but not FAT-formatted.");
    Serial.println("Run Adafruit SPIFlash > fatfs_format first.");

    BTSerial.println("FLASH ERROR: W25Q64JV detected but not FAT-formatted.");
    BTSerial.println("Run Adafruit SPIFlash > fatfs_format first.");
  }

  else if (flashStatus == FLASH_NOT_CHECKED) {
    Serial.println("FLASH ERROR: Flash status was not checked.");

    BTSerial.println("FLASH ERROR: Flash status was not checked.");
  }
}
