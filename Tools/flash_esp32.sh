#!/usr/bin/env bash

BAUD=921600
MINICOM="true"
while getopts ':p:b:m:' OPT; do
    case $OPT in
        p) PORT="$OPTARG";;
        b) BAUD="$OPTARG";;
	m) MINICOM="$OPTARG";;
    esac
done

BIN_FILE_PATH=build/px4_esp32_default
BOOT_FILE_PATH=boards/px4/esp32/extras

BIN_PARTITIONS_FILE=${BOOT_FILE_PATH}/partition-table-esp32.bin
BIN_BOOTLOADER_FILE=${BOOT_FILE_PATH}/bootloader-esp32.bin
BIN_FIRMWARE_FILE=${BIN_FILE_PATH}/px4_esp32_default.bin

esptool.py --chip esp32 -p ${PORT} -b ${BAUD} --before=default_reset --after=hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 ${BIN_BOOTLOADER_FILE} 0x8000 ${BIN_PARTITIONS_FILE} 0x10000 ${BIN_FIRMWARE_FILE}

if [ $MINICOM = "true" ]; then
minicom -D ${PORT} -b 115200
fi
