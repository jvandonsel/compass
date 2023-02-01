#!/bin/bash

SERIAL_DEVICE=/dev/cu.usbserial-0232D38A

which idf.py
if [ "$?" != "0" ]; then
    echo "idf.py not found. Be sure to run '. ~/esp/esp-idf/export.sh'"
    exit -1
fi

esptool.py -p ${SERIAL_DEVICE} -b 460800 --after hard_reset write_flash --flash_mode dio --flash_size detect --flash_freq 26m 0x1000 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/compass.bin
