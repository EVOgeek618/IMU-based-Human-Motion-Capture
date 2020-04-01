#!/usr/bin/env python3
import serial
import time

ser = serial.Serial('/dev/serial/by-path/pci-0000:04:00.3-usb-0:3.4:1.0-port0', 1000000)
T = time.CLOCK_REALTIME
with open('getData.txt', 'w') as file:
	while 1:
		serial_line = str(ser.readline()) +' '+ str(time.clock_gettime(T))
		file.write('\n'+serial_line)

	
