#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>

// MagneticSensorI2C(uint8_t _chip_address, float _cpr, uint8_t _angle_register_msb)
//  chip_address         - I2C芯片地址
//  bit_resolution       - 传感器分辨率
//  angle_register_msb   - 角度读取寄存器msb
//  bits_used_msb        - msb寄存器使用的分辨率位数
MagneticSensorI2C as5600 = MagneticSensorI2C(0x36, 12, 0x0C, 4);
// 快速配置
//MagneticSensorI2C as5600 = MagneticSensorI2C(AS5600_I2C);
//#define AS5600_ADDRESS 0x36
//#define RAW_ANGLE_REGISTER 0x0C
float angle_zero = 0.0;
//float getAngle();

void setup() {
  // 监视点
  Serial.begin(115200);
  as5600.init();

  // 初始化磁性传感器硬件
  //Wire.begin();

  Serial.println("AS5600 ready");
  _delay(3000);
  angle_zero = as5600.getAngle();
  Serial.println(angle_zero);
  _delay(3000);
  }

void loop() {
  // 在终端显示角度和角速度
  //Serial.println(as5600.getAngle() - angle_zero);
  Serial.println(as5600.getSensorAngle());
  
  //Serial.println(as5600.getVelocity());
  as5600.update();
  _delay(100);
}

// float getAngle(){
//   Wire.beginTransmission(AS5600_ADDRESS);
//   Wire.write(RAW_ANGLE_REGISTER);
//   Wire.endTransmission();
//   Wire.requestFrom(AS5600_ADDRESS, 2);
//   if (Wire.available() == 2) {
//     uint16_t rawAngle = Wire.read() << 8 | Wire.read();
//   float angle = rawAngle * 0.00153 - angle_zero; // 转换为角度值
//   }
//   return angle;
// }
// #include <Wire.h>

// #define AS5600_ADDRESS 0x36
// #define RAW_ANGLE_REGISTER 0x0C

// void setup() {
// Serial.begin(115200);
// Wire.begin();
// }

// void loop() {
// Wire.beginTransmission(AS5600_ADDRESS);
// Wire.write(RAW_ANGLE_REGISTER);
// Wire.endTransmission();
// Wire.requestFrom(AS5600_ADDRESS, 2);

// if (Wire.available() == 2) {
// uint16_t rawAngle = Wire.read() << 8 | Wire.read();
// //float angle = rawAngle * 0.0879; // 转换为角度值
// Serial.println(rawAngle);
// }

// delay(100);
// }
