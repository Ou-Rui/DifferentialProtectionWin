SET PATH=D:\Program Files\Keil5\ARM\ARMCC\Bin;C:\Program Files\Common Files\Oracle\Java\javapath;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Windows\System32\OpenSSH\;C:\Program Files (x86)\NVIDIA Corporation\PhysX\Common;C:\Program Files\NVIDIA Corporation\NVIDIA NvDLISR;D:\Program Files\Java\jdk1.8.0_281;D:\Program Files\MATLAB\R2019b\runtime\win64;D:\Program Files\MATLAB\R2019b\bin;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\WINDOWS\System32\WindowsPowerShell\v1.0\;C:\WINDOWS\System32\OpenSSH\;D:\Program Files\Git\cmd;D:\OtherSoftware\netcat-win32-1.12;D:\OtherSoftware\Anacoda3\Scripts;D:\Program Files (x86)\NetSarang\Xftp 7\;D:\OtherSoftware\nodejs\;D:\OtherSoftware\Redis-x64-5.0.10;D:\OtherSoftware\mysql-5.1.38-winx64\bin;D:\OtherSoftware\MinGW\bin\;D:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10\bin;D:\OtherSoftware\OpenOCD-20211116-0.11.0\bin\;D:\Program Files (x86)\NetSarang\Xshell 7\;C:\Users\14475\AppData\Local\Microsoft\WindowsApps;;D:\Program Files\JetBrains\IntelliJ IDEA 2020.3.2\bin;;D:\Program Files\JetBrains\PyCharm 2020.3.3\bin;;D:\Program Files\JetBrains\WebStorm 2020.3.2\bin;;D:\Program Files\JetBrains\GoLand 2021.1.3\bin;;D:\Program Files\Microsoft VS Code\bin;C:\Users\14475\AppData\Roaming\npm
SET CPU_TYPE=STM32F103ZE
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=DifferentialProtection
SET CPU_CLOCK=0x007A1200
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmAsm" --Via "differentialprotection\startup_stm32f103xe._ia"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\main.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\gpio.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\can.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\spi.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\tim.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\usart.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\stm32f1xx_it.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\stm32f1xx_hal_msp.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\stm32f1xx_hal_gpio_ex.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\stm32f1xx_hal_can.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\stm32f1xx_hal.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\stm32f1xx_hal_rcc.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\stm32f1xx_hal_rcc_ex.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\stm32f1xx_hal_gpio.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\stm32f1xx_hal_dma.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\stm32f1xx_hal_cortex.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\stm32f1xx_hal_pwr.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\stm32f1xx_hal_flash.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\stm32f1xx_hal_flash_ex.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\stm32f1xx_hal_exti.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\stm32f1xx_hal_spi.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\stm32f1xx_hal_tim.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\stm32f1xx_hal_tim_ex.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\stm32f1xx_hal_uart.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmCC" --Via "differentialprotection\system_stm32f1xx.__i"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\ArmLink" --Via "DifferentialProtection\DifferentialProtection.lnp"
"D:\Program Files\Keil5\ARM\ARMCC\Bin\fromelf.exe" "DifferentialProtection\DifferentialProtection.axf" --i32combined --output "DifferentialProtection\DifferentialProtection.hex"
