# This file should be used to debug the quaternion data 
# sent by the RP2040 (Pi Pico) over serial. Edit this file 
# to use the serial port your Pico is on, then run it with 
# `python scripts/debug-quaternion.py` once your Pico is plugged in via USB
import sys
import struct
# import time
import serial

PICO_SERIAL_PORT = '/dev/cu.usbmodemPI_PICO1'

def log(msg):
  sys.stdout.write('\r{}'.format(msg))
  sys.stdout.flush()

try:
  # start = time.time()
  # Serial takes these two parameters: serial device and baudrate
  # Idk why this takes 40+ seconds to run / open
  serial_conn = serial.Serial(PICO_SERIAL_PORT, 19200)
  # print(time.time() - start)
  # exit()
  while True:
    quaternion_data = serial_conn.read(size=16)
    quaternion_data_type = type(quaternion_data)
    quaternion_data_length = len(quaternion_data)
    w = struct.unpack('f', quaternion_data[0:4])
    x = struct.unpack('f', quaternion_data[4:8])
    y = struct.unpack('f', quaternion_data[8:12])
    z = struct.unpack('f', quaternion_data[12:16])
    log('W: {} X: {} Y: {} Z: {}'.format(
        w[0],
        x[0],
        y[0],
        z[0]
      )
    )
except KeyboardInterrupt:
  serial_conn.close()