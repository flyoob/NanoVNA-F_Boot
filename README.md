# NanoVNA-F_Boot
[![GitHub release](https://img.shields.io/github/release/flyoob/NanoVNA-F_Boot.svg?style=flat)][release]
[release]: https://github.com/flyoob/NanoVNA-F_Boot/releases

NanoVNA-F  引导程序，像 U 盘拷贝文件一样升级固件。
NanoVNA-F BootLoader, STM32F1+SPI FLASH+USB Device Mass Storage for IAP.

Created by STM32CubeMX 4.27.0 V1.0
* HAL Lib  ：STM32Cube_FW_F1_V1.6.1
* MDK Ver  ：uVision V5.23.0.0
* MDK Pack : ARM::CMSIS Ver: 5.2.0(2017-11-16)
* MDK Pack : Keil::STM32F1xx_DFP Ver: 2.3.0(2018-11-05)
* FatFs    ：R0.11 (February 09, 2015)
* CHIP
STM32F103VET6 FLASH: 512 KB, SRAM: 64 KB
* SPI Flash
W25Q128JVSIQTR

HAL Lib Path: C:/Users/S04/STM32Cube/Repository/STM32Cube_FW_F1_V1.6.1

### 如何使用 / How to use
1. 使用 Type-C 将设备连接到 USB，在开机的同时按下波轮。
   Connect the device to USB using Type-C and press the pulsator while powering up.
![1](/Img/STM32.jpg)

2. 电脑端显示为一个 U 盘，将 update.bin 拷入。
   At PC side, act as a Udisk. Copy you update.bin into it.
![2](/Img/PC.jpg)

3. 重启设备，自动运行 APP。
   Restart the device and run the app automatically.
![3](/Img/STM32_RUNAPP.jpg)

### MDK-ARM 工程编译 / Build by MDK-ARM
根据实际情况修改库文件路径。
Change the library file path according to the actual situation.
![4](/Img/MDK-ARM.jpg)

### SW4STM32 工程编译 / Build by SW4STM32
待完善 / Coomming soon
