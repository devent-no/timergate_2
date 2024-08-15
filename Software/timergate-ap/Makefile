#. $HOME/esp/esp-idf/export.sh

DEV=/dev/ttyACM1

build:
	idf.py build

prepare:
	. /home/elias/esp/esp-idf/export.sh

target:
	idf.py set-target esp32s3

flash:
	idf.py flash -p ${DEV}

monitor:
	idf.py monitor -p ${DEV}

config:
	idf.py menuconfig


add-dependency:
	idf.py add-dependency "espressif/led_strip^2.5.3"
	idf.py update-dependencies
	
.PHONY: build
