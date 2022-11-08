This project is made for eclipse-embedcpp-2022-09-R-linux-gtk-x86_64 and xpack-arm-none-eabi-gcc-11.3.1-1.1-linux-x64
STM32CubeProgrammer (ver 2.9) may be used for flashing but Reset mode -> Hardware reset (because this program reinit SWCLK and SWDIO pins)
Port from STM32F405RGT6 to GD32F405RGT6
STM32CubeProgrammer may be used, but for THIS program, in STM32CubeProgrammer must be select Reset mode -> Hardware reset (cheap chinese Stlinkv2 clones does not drives hw Reset!!!)
st-tools from https://github.com/stlink-org/stlink may be used to flash THIS program, but only one time. Seems that st-flash does not drive  RESET pin?
BUT if GD32F405 bootloader is started (BOOT0 pin at 3.3v), then st-flash may work with SWDIO SWCLK pins (without RESET), and therefore any st-link clone may be used.
