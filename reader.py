#!/usr/bin/python
import serial
import signal
import atexit
import os

LOG_FILE='./messages.log'
USB_FILE='/dev/ttyUSB0'
BAUD=9600

print "reading serial device"
ser = serial.Serial(USB_FILE, BAUD)

print "opening output file"
try:
  os.remove(LOG_FILE)
except OSError:
  pass

output = open(LOG_FILE, 'w')

def cleanup():
  # close serial connection
  ser.close()
  # close outfile
  output.close()

if __name__ == "__main__":
  atexit.register(cleanup)
  signal.signal(signal.SIGTERM, lambda signum, stack_frame: exit(1))
  print "ctrl+c to quit"

  # throw this away
  data = ser.readline()

  while True:
    data = ser.readline()
    output.write(data) 
    output.flush()
