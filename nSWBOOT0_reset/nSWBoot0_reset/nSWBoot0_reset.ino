#include <Arduino.h>

#define FLASH_KEY1 0x45670123U
#define FLASH_KEY2 0xCDEF89ABU
#define FLASH_OPTKEY1 0x08192A3BU
#define FLASH_OPTKEY2 0x4C5D6E7FU

void setup() {
Serial.begin(115200);
Serial.println("Modifying nSWBOOT0...");

// 解锁 FLASH 寄存器
FLASH->KEYR = FLASH_KEY1;
FLASH->KEYR = FLASH_KEY2;

// 解锁选项字节寄存器
FLASH->OPTKEYR = FLASH_OPTKEY1;
FLASH->OPTKEYR = FLASH_OPTKEY2;

// 检查解锁成功
if (!(FLASH->CR & FLASH_CR_OPTLOCK)) {
Serial.println("Option bytes unlocked!");

// 读取当前的选项字节配置
uint32_t userOptionBytes = FLASH->OPTR;

// 修改 nSWBOOT0 位：清零以忽略 BOOT0 引脚
userOptionBytes &= ~FLASH_OPTR_nSWBOOT0;

// 写回新的选项字节
FLASH->CR |= FLASH_CR_OPTSTRT; // 启动选项字节编程
FLASH->OPTR = userOptionBytes;

// 等待操作完成
while (FLASH->SR & FLASH_SR_BSY);

// 锁定选项字节寄存器
FLASH->CR |= FLASH_CR_OPTLOCK;

// 系统复位以使设置生效
Serial.println("nSWBOOT0 modified. Resetting system...");
NVIC_SystemReset();
} else {
Serial.println("Failed to unlock option bytes.");
}
}

void loop() {
// 空循环
}