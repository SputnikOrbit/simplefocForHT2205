/**
  **************************** 2025 Theseus Mecanum****************************
  * @file       simpleFOC2205.ino
  * @brief      This is the main file for the HT2205 gimbal motor control.
  * @note       This file is based on the SimpleFOC library. 
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025/02/06     SPUTNIK         1. Distributed version
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  **************************** COPYRIGHT 2025 Theseus Mecanum****************************
  */

#include <SimpleFOC.h>
/**************************************************/
/*固有参数表，每个电调需要调整的内容*/
#define MEC_NAME "Mouse0"
#define MOTOR_CODE "M2"

Direction motor_dir = Direction::CW;
float zero_electric_angle = 1.08;
/**************************************************/


const char* buildDate = __DATE__;
const char* buildTime = __TIME__;
const char* monthNames[] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun",
                             "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };


BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

/**************************************************/
const int pwmPin = PA15;
const int pwmMin = 800;
const int pwmMax = 2200;
int pwmControlMin = 1100;
int pwmControlMax = 1900;
const int pwmTrim = 1500;

const float speedMin = -25.0;
const float speedMax = 25.0;

unsigned int pulse_start_time = 0;
unsigned int pulse_width = 0;
unsigned int raw_pulse = 0;

float target_velocity = 0.0;
float previousPwmValue = 0.0;
float alpha = 0.4;
int filteredPwmValue = 0;

unsigned long lastDebugTime = 0;
const unsigned long debugInterval = 1000;
unsigned long loop_time = 0;
char formattedDateTime[20];
/**************************************************/

void setup() {

  SystemClock_Config();

  Serial.begin(115200);

  sensor.init();

  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.pwm_frequency = 20000;  //set pwm frequency for lower heat
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

  motor.sensor_direction = motor_dir;
  motor.zero_electric_angle = zero_electric_angle;

  motor.init();
  motor.initFOC();
  motor.enable();
  _delay(1000);

  pinMode(PA15, INPUT);
  attachInterrupt(digitalPinToInterrupt(PA15), pulseHandler, CHANGE);

  // 获取编译日期的各个部分
  int day = atoi(&buildDate[4]);
  int year = atoi(&buildDate[7]);
  int month = getMonth(buildDate);
  int hour = atoi(&buildTime[0]);
  int minute = atoi(&buildTime[3]);
  sprintf(formattedDateTime, "%04d/%02d/%02d/%02d:%02d", year, month, day, hour, minute);
}


void loop() {
  float filteredPwmValue = lowPassFilter(pulse_width, previousPwmValue, alpha);
  previousPwmValue = filteredPwmValue;

  if (filteredPwmValue >= pwmMin && filteredPwmValue <= pwmMax) {
    if (abs(filteredPwmValue - pwmTrim) < 30) {
      target_velocity = 0.0;  // Dead zone
    } else if (filteredPwmValue < pwmTrim) {
      target_velocity = mapFloat(filteredPwmValue, pwmControlMin, pwmTrim, speedMin, 0);
    } else {
      target_velocity = mapFloat(filteredPwmValue, pwmTrim, pwmControlMax, 0, speedMax);
    }

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


    Serial.print(MEC_NAME);
    Serial.print(" ");
    Serial.print(MOTOR_CODE);
    Serial.print(" ");
    if (motor_dir == Direction::CW)
      Serial.print("CW");
    else
      Serial.print("CCW");
    Serial.print(" enc ");
    Serial.println(motor.zero_electric_angle);
    Serial.println(formattedDateTime);

    lastDebugTime = currentTime;
  }
  loop_time = currentTime;
}


void pulseHandler() {  //中断处理函数

  if (digitalRead(pwmPin) == HIGH) {  //高电平开始计时
    pulse_start_time = micros();
  } else {
    raw_pulse = micros() - pulse_start_time;
    if (raw_pulse > pwmMin && raw_pulse < pwmMax)
      pulse_width = raw_pulse;
  }
}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {  //浮点数map
  return out_min + (out_max - out_min) * ((x - in_min) / (in_max - in_min));
}



float lowPassFilter(float currentValue, float previousValue, float alpha) {  //低通滤波
  return alpha * currentValue + (1 - alpha) * previousValue;
}


void SystemClock_Config(void) {

  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  //实测最低正常运行时钟频率

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;  //开启HSI和HSI48
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;  //为usb准备
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;  //锁相环
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;  // 16M / 2 == 8M
  RCC_OscInitStruct.PLL.PLLN = 12;             // 8M * 12 == 96M
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;  //96M / 2 == 48M
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  // 48M
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK) {
    Error_Handler();
  }
}


int getMonth(const char* date) {
  for (int i = 0; i < 12; i++) {
    if (strncmp(date, monthNames[i], 3) == 0) {
      return i + 1;  // 月份从 1 开始
    }
  }
  return 0;  // 错误情况
}