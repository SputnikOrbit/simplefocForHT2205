在适配HT2205电机与simplefoc控制板时使用的代码。

![image-20241122144849687](README.assets/image-20241122144849687.png)向hfoc控制板中最终烧录的是simpefocFor2205Debug，因为debug保留有串口输出，可以减少调参和排障工作量。

![image-20241122143532154](README.assets/image-20241122143532154.png)

采用它们做了轮毂电机，扭矩很小，但得以于载具并不大的重量和较低的滚阻，仍然能达到很快的速度。

stlink的3v3最好不要给芯片供电，防止任何潜在的危险。

![image-20241122143741572](README.assets/image-20241122143741572.png)

不专业且缺乏安全措施的操作，pwm信号也受到较大干扰，但还是坚持到做完了。下一步必须优化可靠性了。

![image-20241122143846113](README.assets/image-20241122143846113.png)

需要降低驱动板芯片运行的时钟频率，目前stm32G431速度很快运行起来温度很高。

# 2月6日修改：

```cpp
void pulseHandler() {
  
  if (digitalRead(pwmPin) == HIGH) {
    pulse_start_time = micros();  
  } else {
    raw_pulse = micros() - pulse_start_time;
    if (raw_pulse > pwmMin && raw_pulse < pwmMax) 
      pulse_width = raw_pulse;
  }
}
```

ubuntu 上 G431文件存储位置：

/home/sputnik/.arduino15/packages/STMicroelectronics/hardware/stm32/2.8.1/variants/STM32G4xx/G431C(6-8-B)U_G441CBU

而可用的编译环境（由于路径导致hal库找不到的问题通过放置在编译环境中解决）在

~/Arduino中，HT2205的foc修改版在simplefoc的examples中