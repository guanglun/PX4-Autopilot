#!/usr/bin/env bash

BAUD=921600
MINICOM="true"
PORT="/dev/ttyACM0"
FLASH_BOOT_FILE="false"

BIN_FILE_PATH=build/px4_esp32_default
BOOT_FILE_PATH=boards/px4/esp32/boot

BIN_PARTITIONS_FILE=${BOOT_FILE_PATH}/partition-table.bin
BIN_BOOTLOADER_FILE=${BOOT_FILE_PATH}/bootloader.bin
BIN_FIRMWARE_FILE=${BIN_FILE_PATH}/px4_esp32_default.bin

while getopts ':p:b:m:f:t:h:' OPT; do
    case $OPT in
        p) PORT="$OPTARG";;
        b) BAUD="$OPTARG";;
	m) MINICOM="$OPTARG";;
	f) FLASH_BOOT_FILE="$OPTARG";;
	t) BIN_FIRMWARE_FILE="$OPTARG";;
	h) echo "-p:port -b:baud -m:";;
    esac
done

if [ $FLASH_BOOT_FILE = "true" ]; then
esptool.py --chip esp32 -p ${PORT} -b ${BAUD} --before=default_reset --after=hard_reset write_flash --flash_mode "dio" --flash_freq "80m" --flash_size "4MB" 0x1000 ${BIN_BOOTLOADER_FILE} 0x8000 ${BIN_PARTITIONS_FILE} 0x10000 ${BIN_FIRMWARE_FILE}
else
esptool.py --chip esp32 -p ${PORT} -b ${BAUD} --before=default_reset --after=hard_reset write_flash --flash_mode "dio" --flash_freq "80m" --flash_size "4MB" 0x10000 ${BIN_FIRMWARE_FILE}
fi


if [ $MINICOM = "true" ]; then
minicom -D ${PORT} -b 115200
fi
