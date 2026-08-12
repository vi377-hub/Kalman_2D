#include <Wire.h>

const uint8_t ICMAddress = 0x68;

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float AngleRoll_kalman, AnglePitch_kalman;

const float alpha = 0.85;
const float roll_offset = 1.83;
const float pitch_offset = 1.18;

const float Q_angle = 0.02;
const float Q_bias = 0.003;
const float R_measure = 0.5;

float dt = 0.001;
unsigned long last_timer;
bool ImuDataReady = false;

struct Kalman2D {
  float Angle;
  float Bias;
  float P[2][2];
};

Kalman2D KalmanRoll = {0.0, 0.0, {{1.0, 0.0}, {0.0, 1.0}}};
Kalman2D KalmanPitch = {0.0, 0.0, {{1.0, 0.0}, {0.0, 1.0}}};

void write_register(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(ICMAddress);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t read_register(uint8_t reg) {
  Wire.beginTransmission(ICMAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;

  if (Wire.requestFrom(ICMAddress, (uint8_t)1) != 1) return 0xFF;
  return Wire.read();
}

void gyro_signals(void) {
  ImuDataReady = false;

  Wire.beginTransmission(ICMAddress);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom(ICMAddress, (uint8_t)14) != 14) return;

  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.read();
  Wire.read();

  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  float AccX_raw = (float)AccXLSB / 4096.0;
  float AccY_raw = (float)AccYLSB / 4096.0;
  float AccZ_raw = (float)AccZLSB / 4096.0;

  static bool AccStarted = false;
  if (!AccStarted) {
    AccX = AccX_raw;
    AccY = AccY_raw;
    AccZ = AccZ_raw;
    AccStarted = true;
  } else {
    AccX = alpha * AccX + (1.0 - alpha) * AccX_raw;
    AccY = alpha * AccY + (1.0 - alpha) * AccY_raw;
    AccZ = alpha * AccZ + (1.0 - alpha) * AccZ_raw;
  }

  AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 180.0 / PI + roll_offset;
  AnglePitch = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI + pitch_offset;

  ImuDataReady = true;
}

float kalman_2d(Kalman2D &Filter, float Rate, float AngleMeasurement, float R_measure_now) {
  Rate -= Filter.Bias;
  Filter.Angle += Rate * dt;

  Filter.P[0][0] += dt * (dt * Filter.P[1][1] - Filter.P[0][1] - Filter.P[1][0] + Q_angle);
  Filter.P[0][1] -= dt * Filter.P[1][1];
  Filter.P[1][0] -= dt * Filter.P[1][1];
  Filter.P[1][1] += Q_bias * dt;

  float Innovation = AngleMeasurement - Filter.Angle;
  float InnovationCovariance = Filter.P[0][0] + R_measure_now;
  float KalmanGain[2];

  KalmanGain[0] = Filter.P[0][0] / InnovationCovariance;
  KalmanGain[1] = Filter.P[1][0] / InnovationCovariance;

  Filter.Angle += KalmanGain[0] * Innovation;
  Filter.Bias += KalmanGain[1] * Innovation;

  float P00_temp = Filter.P[0][0];
  float P01_temp = Filter.P[0][1];

  Filter.P[0][0] -= KalmanGain[0] * P00_temp;
  Filter.P[0][1] -= KalmanGain[0] * P01_temp;
  Filter.P[1][0] -= KalmanGain[1] * P00_temp;
  Filter.P[1][1] -= KalmanGain[1] * P01_temp;

  return Filter.Angle;
}

void kalman(void) {
  float AccTotal = sqrt(AccX * AccX + AccY * AccY + AccZ * AccZ);
  float R_measure_now = R_measure;
  if (fabs(AccTotal - 1.0) > 0.15) R_measure_now = R_measure * 10.0;

  AngleRoll_kalman = kalman_2d(KalmanRoll, RateRoll, AngleRoll, R_measure_now);
  AnglePitch_kalman = kalman_2d(KalmanPitch, RatePitch, AnglePitch, R_measure_now);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  delay(250);

  write_register(0x6B, 0x80);
  delay(100);
  write_register(0x6B, 0x01);
  delay(10);

  uint8_t WhoAmI = read_register(0x75);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(WhoAmI, HEX);

  if (WhoAmI != 0x12) {
    Serial.println("ICM20602 not found");
    while (1) delay(1000);
  }

  write_register(0x1A, 0x05);
  write_register(0x1C, 0x10);
  write_register(0x1B, 0x08);

  while (RateCalibrationNumber < 2000) {
    gyro_signals();
    if (ImuDataReady) {
      RateCalibrationRoll += RateRoll;
      RateCalibrationPitch += RatePitch;
      RateCalibrationYaw += RateYaw;
      RateCalibrationNumber++;
    }
    delay(1);
  }

  RateCalibrationRoll /= 2000.0;
  RateCalibrationPitch /= 2000.0;
  RateCalibrationYaw /= 2000.0;

  gyro_signals();
  KalmanRoll.Angle = AngleRoll;
  KalmanPitch.Angle = AnglePitch;
  KalmanRoll.Bias = RateCalibrationRoll;
  KalmanPitch.Bias = RateCalibrationPitch;
  AngleRoll_kalman = AngleRoll;
  AnglePitch_kalman = AnglePitch;

  last_timer = micros();
}

void loop() {
  unsigned long LoopTimer = micros();

  gyro_signals();
  if (!ImuDataReady) return;

  dt = (LoopTimer - last_timer) / 1000000.0;
  last_timer = LoopTimer;
  if (dt <= 0.0 || dt > 0.02) dt = 0.001;

  kalman();

  // Serial.print("Roll: ");
  // Serial.print(AngleRoll_kalman);
  // Serial.print("\tRoll bias: ");
  // Serial.print(KalmanRoll.Bias);
  // Serial.print("\tPitch: ");
  // Serial.print(AnglePitch_kalman);
  // Serial.print("\tPitch bias: ");
  // Serial.println(KalmanPitch.Bias);
}
