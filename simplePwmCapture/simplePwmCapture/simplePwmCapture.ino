// #include <HardwareTimer.h>

// HardwareTimer *MyTim;

// volatile uint32_t Period = 0;
// volatile uint32_t Pulse = 0;

// void setup() {
//   Serial.begin(115200);
//   delay(1000);
//   Serial.println("I drive1");

//   // MyTim = new HardwareTimer(TIM2);
//   // Serial.println("I drive2");
//   // MyTim->setMode(1, TIMER_INPUT_FREQ_DUTY_MEASUREMENT, PA15);
//   // Serial.println("I drive3");
//   // MyTim->attachInterrupt(1, rising);
//   // Serial.println("I drive4");
//   // MyTim->attachInterrupt(2, falling);
//   // Serial.println("I drive5");
//   // MyTim->resume();

//   Serial.println("And i pass");
//   delay(1000);
// }



// void loop() {
//   pwmValue = pulseIn(pwmPin, HIGH, 25000);
//   Serial.println(Pulse);
// }

// // void rising() {
// //   Period = MyTim->getCaptureCompare(1);
// // }

// // void falling() {
// //   Pulse = MyTim->getCaptureCompare(2);
// // }
volatile unsigned long pulse_start_time = 0;
volatile unsigned long pulse_width = 0;

void pulseHandler() {
static bool rising = true;
if (rising) {
pulse_start_time = micros(); // 上升沿，记录开始时间
} else {
pulse_width = micros() - pulse_start_time; // 下降沿，计算脉宽
}
rising = !rising;
}


void setup() {
  pinMode(PA15, INPUT);
  attachInterrupt(digitalPinToInterrupt(PA15), pulseHandler, CHANGE);

  Serial.begin(115200);
  delay(1000);
  Serial.println("I drive1");
}

void loop() {
// 继续处理电机控制逻辑

  Serial.println(pulse_width);
// 使用 `pulse_width` 来获取脉冲持续时间
}

