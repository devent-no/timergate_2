#. $HOME/esp/esp-idf/export.sh

DEV=/dev/ttyACM3

build:
	idf.py build

flash:
	idf.py flash -p ${DEV}

monitor:
	idf.py monitor -p ${DEV}

menuconfig:
	idf.py menuconfig

.PHONY: build
