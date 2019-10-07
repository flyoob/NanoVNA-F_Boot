# NanoVNA-F_Boot

[![GitHub release](https://img.shields.io/github/release/flyoob/NanoVNA-F_Boot.svg?style=flat)][release]

[release]: https://github.com/flyoob/NanoVNA-F_Boot/releases

NanoVNA-F  引导程序，简单3步，像 U 盘拷贝文件一样升级固件。
NanoVNA-F BootLoader, STM32F1+SPI FLASH+USB Device Mass Storage for IAP, very simple way to upgrade your app.

### 如何使用 / How to use
1. 使用 Type-C 将设备连接到 USB，在开机前按下波轮中键，并保持，然后开机。  
   液晶屏显示如下提示，代表已经进入Bootloader。  
   Connect the device to USB using Type-C, press the pulsator and hold on, then powering up the device.  
   The LCD displays the following prompt, indicating that the bootloader has been entered.  
![1](/Img/STM32.jpg)

2. 随后，NanoVNA-F在电脑端显示为一个 U 盘，将新的 update.bin 拷入。  
   Then at PC side, device act as a Udisk. Copy you new update.bin into it.  
![2](/Img/PC_Udisk.png)
![3](/Img/Copy_bin.jpg)
![4](/Img/Copy_bin_1.jpg)

3. 设备重新上电，自动运行 APP。  
   Re-power the device and run the app automatically.  
![5](/Img/STM32_RUNAPP.jpg)

### MDK-ARM 工程编译 / Build by MDK-ARM
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

根据实际情况修改库文件路径。  
Change the library file path according to the actual situation.  
![6](/Img/MDK-ARM.jpg)

### SW4STM32 工程编译 / Build by SW4STM32
待完善 / Coming soon
