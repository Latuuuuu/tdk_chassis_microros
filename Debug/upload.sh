#!/bin/bash

# 设定 STM32F446RE 程序路径
PROGRAM_PATH="./tdk_chassis_microros.elf"

# 设定 STM32F446RE 起始地址，通常是 0x08000000
FLASH_ADDRESS=0x08000000

# OpenOCD 配置文件路径
INTERFACE_CFG="interface/stlink-v2.cfg"
TARGET_CFG="target/stm32f4x.cfg"

# 检查程序文件是否存在
if [ ! -f "$PROGRAM_PATH" ]; then
  echo "程序文件 $PROGRAM_PATH 不存在！请检查路径。"
  exit 1
fi

# 启动 OpenOCD
echo "启动 OpenOCD..."
openocd -f $INTERFACE_CFG -f $TARGET_CFG -c "init; reset halt; flash erase_sector 0 0 0; flash write_image erase $PROGRAM_PATH $FLASH_ADDRESS elf; reset run; exit"

# 完成烧录后，显示提示信息
echo "烧录完成，STM32F446RE 已重启并开始运行程序。"
