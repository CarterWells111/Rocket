#include <I2Cdev.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Adafruit_BMP280.h>

MPU6050 mpu;
Adafruit_BMP280 bmp;
const int relayPin = 9;  // Relay control pin

const float AScale = 16.0 / 32768.0 * 9.81; // ±16g 对应的比例因子
const float GScale = 250.0 / 32768.0 * (PI / 180.0); // ±250°/s 对应的比例因子

unsigned long currentTime, lastTime;
float deltaTime;

int16_t ax, ay, az, gx, gy, gz;
float pitch, roll, yaw;
float q[4] = {1.0, 0.0, 0.0, 0.0}; // 初始化四元数

float InitPressure = 0;  // 存储发射时的基准气压
float AccZ = 0;
float AltBMP = 0;
float AltMPU = 0;
float velocity = 0;
long ax_offset, ay_offset, az_offset;
long gx_offset, gy_offset, gz_offset;
long yaw_offset, pitch_offset, roll_offset;

float Kp = 30.0f; // 比例增益
float Ki = 0.01f;  // 积分增益

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  Wire.begin();
  delay(5000);
  Init_MPU_6050();
  Init_BMP280();
  lastTime = millis();
  Serial.println("Preparation done! Ready to fly!");
  Serial.println("Time   Altitude            Acceleration                attitude");
}

void loop() {
  if (millis() - lastTime >= 10) {
    currentTime = millis();
    deltaTime = (float)(currentTime - lastTime) / 1000.00;  // get deltaT
    lastTime = currentTime;
  
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);
    ax = (ax - ax_offset) * AScale;
    ay = (ay - ay_offset) * AScale;
    az = (az - az_offset) * AScale;
    gx = (gx - gx_offset) * GScale;
    gy = (gy - gy_offset) * GScale;
    gz = (gz - gz_offset) * GScale;
  
    Mahony_update(ax, ay, az, gx, gy, gz, deltaTime);
  
    // 更新 BMP 高度
    AltBMP = bmp.readAltitude(InitPressure);  // 传入基准气压计算高度
  
    yaw = CurrentInclination(0) - yaw_offset;
    pitch = CurrentInclination(1) - pitch_offset;
    roll = CurrentInclination(2) - roll_offset;
    
    // 将数据写入黑匣子模块
    Serial.print(millis()/1000.0);
    Serial.print("s, AltitudeBMP: ");
    Serial.print(AltBMP);
    Serial.print("m, Ax: ");
    Serial.print(ax);
    Serial.print(", Ay: ");
    Serial.print(ay);
    Serial.print(", Az: ");
    Serial.print(az);
    Serial.print(", Yaw: ");
    Serial.print(yaw);
    Serial.print(", Pitch: ");
    Serial.print(pitch);
    Serial.print(", Roll: ");
    Serial.println(roll);

    if((yaw < -10 || yaw > 10 || pitch < -10 || pitch > 10) && az < 0 && millis() > 10000)
    {
      yaw = CurrentInclination(0) - yaw_offset;
      pitch = CurrentInclination(1) - pitch_offset;
      roll = CurrentInclination(2) - roll_offset;
      mpu.getAcceleration(&ax, &ay, &az);
      az = (az - az_offset) * AScale;
      if((yaw < -10 || yaw > 10 || pitch < -10 || pitch > 10) && az < 0 && millis() > 10000) //二次判断防止误开
      {
        digitalWrite(relayPin, HIGH); // 释放降落伞
        Serial.println("Parachute released!");
        } 
    }
  }
}

void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  //error terms
  float qa, qb, qc;
  static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
  float tmp;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  tmp = ax * ax + ay * ay + az * az;
  if (tmp > 0.0)
  {

    // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
    recipNorm = 1.0 / sqrt(tmp);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity in the body frame (factor of two divided out)
    vx = q[1] * q[3] - q[0] * q[2];
    vy = q[0] * q[1] + q[2] * q[3];
    vz = q[0] * q[0] - 0.5f + q[3] * q[3];

    // Error is cross product between estimated and measured direction of gravity in body frame
    // (half the actual magnitude)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // Compute and apply to gyro term the integral feedback, if enabled
    if (Ki > 0.0f) {
      ix += Ki * ex * deltat;  // integral error scaled by Ki
      iy += Ki * ey * deltat;
      iz += Ki * ez * deltat;
      gx += ix;  // apply integral feedback
      gy += iy;
      gz += iz;
    }

    // Apply proportional feedback to gyro term
    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;
  }

  // Integrate rate of change of quaternion, q cross gyro term
  deltat = 0.5 * deltat;
  gx *= deltat;   // pre-multiply common factors
  gy *= deltat;
  gz *= deltat;
  qa = q[0];
  qb = q[1];
  qc = q[2];
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  // renormalise quaternion
  recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] = q[0] * recipNorm;
  q[1] = q[1] * recipNorm;
  q[2] = q[2] * recipNorm;
  q[3] = q[3] * recipNorm;
}

// Function to get current inclination
float CurrentInclination(int axis) {
  switch(axis) {
      case 0: // Yaw
          return atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) * 180.0 / M_PI;
      case 1: // Pitch
          return asin(2.0f * (q[0] * q[2] - q[3] * q[1])) * 180.0 / M_PI;
      case 2: // Roll
          return atan2(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) * 180.0 / M_PI;
      default:
          return 0.0f; // Invalid axis
  }
}

// Function to initialize MPU6050
void Init_MPU_6050() {
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
    mpu.setFullScaleGyroRange(0);
    mpu.setFullScaleAccelRange(3);
    Serial.println("MPU6050 initialized successfully!");
}

// Function to initialize BMP280
void Init_BMP280() {
    if (bmp.begin(0x76)) {
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                        Adafruit_BMP280::SAMPLING_X2,
                        Adafruit_BMP280::SAMPLING_X16,
                        Adafruit_BMP280::FILTER_X4,
                        Adafruit_BMP280::STANDBY_MS_1);
        InitPressure = bmp.readPressure() / 100.0F;  // 转换为 hPa（1 Pa = 0.01 hPa）
        Serial.print("Initial Pressure: ");
        Serial.println(InitPressure);
        Serial.println("BMP280 initialized successfully!");
    } else {
        Serial.println("BMP280 initialization failed!");
    }
}