#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

//pwm controller
const int pwmPin = PA15;
const int pwmMin = 800;
const int pwmMax = 2200;
int pwmControlMin = 1000;
int pwmControlMax = 2000;
const int pwmTrim = 1500;
const int pwmMidTrim = 18;

const float speedMin = -25.0;
const float speedMax = 25.0;

volatile unsigned long pulse_start_time = 0;
volatile unsigned long pulse_width = 0;
volatile unsigned long raw_pulse = 0;

void pulseHandler() {
  static bool rising = true;
  if (rising) {
  pulse_start_time = micros(); // 上升沿，记录开始时间
} else {
  int tmp_value = micros() - pulse_start_time; //防止pwm读取时序误码
  raw_pulse = tmp_value;
  int res_value = 20000.0 + pwmMidTrim - tmp_value;
  if((tmp_value > pwmMin) && (tmp_value < pwmMax))
    pulse_width = tmp_value;
  else if ((res_value > pwmMin) && (res_value < pwmMax))
    pulse_width = res_value;
  else
    pulse_width = 0;
}
  rising = !rising;
}

void setup() {
  Serial.begin(115200);

  sensor.init();

  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  currentSense.linkDriver(&driver);
  currentSense.init();
  currentSense.skip_align = true;
  motor.linkCurrentSense(&currentSense);

  motor.voltage_sensor_align = 1;
  motor.velocity_index_search = 3;
  motor.voltage_limit = 6;
  motor.velocity_limit = 1000;

  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::foc_current;

  motor.PID_current_q.P = 0.5;
  motor.PID_current_q.I = 9;

  motor.current_limit = 1.2;

  motor.PID_velocity.P = 0.8;
  motor.PID_velocity.I = 1.2;
  // motor.PID_velocity.D = 0.0001;
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.01;

  motor.P_angle.P = 20;

  motor.sensor_direction = Direction::CCW;
  motor.zero_electric_angle = 1.75;

  motor.init();
  motor.initFOC();
  motor.enable();
  _delay(1000);

  pinMode(PA15, INPUT);
  attachInterrupt(digitalPinToInterrupt(PA15), pulseHandler, CHANGE);
}

float target_velocity = 0.0;
float previousPwmValue = 0.0;
float alpha = 0.4;
int filteredPwmValue = 0;
unsigned long lastDebugTime = 0;
const unsigned long debugInterval = 1000;
unsigned long loop_time = 0;

void loop() {
  float filteredPwmValue = lowPassFilter(pulse_width, previousPwmValue, alpha);
  previousPwmValue = filteredPwmValue;

  if (filteredPwmValue >= pwmMin && filteredPwmValue   <= pwmMax) {
    if (abs(filteredPwmValue - pwmTrim) < 30) {
      target_velocity = 0.0;  // Dead zone
    } else if (filteredPwmValue < pwmTrim) {
      target_velocity = map(filteredPwmValue, pwmControlMin, pwmTrim, speedMin, 0);
    } else {
      target_velocity = map(filteredPwmValue, pwmTrim, pwmControlMax, 0, speedMax);
    }

    //Serial.println(filteredPwmValue);
  } else {
    target_velocity = 0.0;  //stay for commands wo electricity
  }
  motor.loopFOC();
  motor.move(target_velocity);

  // 仅在指定的时间间隔内输出调试信息
  unsigned long currentTime = millis();
  if (currentTime - lastDebugTime >= debugInterval) {
    loop_time = millis() - loop_time;
    Serial.print("Raw PWM: ");
    Serial.println(raw_pulse);
    Serial.print("PWM Value: ");
    Serial.println(pulse_width);
    Serial.print("TaV: ");
    Serial.println(target_velocity);
    //固有参数日志

    
    Serial.print("Mouse1 M3 CCW enc");
    Serial.println(motor.zero_electric_angle);
    Serial.print("MidTrim ");
    Serial.println(pwmMidTrim);
    Serial.println("2024 11 21");

    lastDebugTime = currentTime;
  }
  loop_time = currentTime;

}

float lowPassFilter(float currentValue, float previousValue, float alpha) {
  return alpha * currentValue + (1 - alpha) * previousValue;
}