#!/bin/sh
killall st-util
make
st-flash --reset write build/firmware.bin 0x8000000
