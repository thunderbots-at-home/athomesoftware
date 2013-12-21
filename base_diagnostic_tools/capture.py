#!/usr/bin/python
import serial
import signal
import atexit
import os
import optparse
import time

###################################################
##                   USB PORTS                   ##
## To use this script, the PORT variable needs to
## be set to the name of your desired port.  To
## make it easier, template names have been
## provided for Windows, Mac, and Linux.
## Uncomment the one you want and change it.
##
## Your Arduino's port name can be found in the
## Arduino IDE under Tools > Serial Port.
##
## Please do not unecessarily commit
## your port name changes!
#
# Windows
# PORT = 'COM10'
#
# Mac
PORT = '/dev/tty.usbmodemfd121'
#
# Linux
# PORT = 'dev/ttyUSB0'
#
###################################################

# Generate filename based on current time
currentTime = time.localtime()
formattedTime = time.ctime()

g = []

for i in range(0,len(currentTime)-1):
  g.append(str(currentTime[i]).zfill(2))

filename = ( "DRIVETRAIN_LOG_" + g[0] + g[1] + g[2] +
"_" + g[3] + g[4] + g[5] + ".csv" )

try:
  os.makedirs('Log Files/')
except OSError:
  pass


# parse input
parser = optparse.OptionParser()

parser.add_option('-o', '--output',
    action='store', dest='output',
    help='output file', default='Log Files/' + filename)

parser.add_option('-i', '--input',
    action='store', dest='input',
    help='usb file to read',
    default=PORT)

parser.add_option('-b', '--baud',
    action='store', dest='baud',
    help='baud rate for reading device file',
    default='9600')

parser.add_option('-s', '--init',
    action='store', dest='init',
    help='initialization line to send to arduino',
    default='default')

options, args = parser.parse_args()

print "reading serial device"
ser = serial.Serial(options.input, options.baud)

print "opening output file"
try:
  os.remove(options.output)
except OSError:
  pass

output = open(options.output, 'w')


def cleanup():
  # close serial connection
  ser.close()
  # close outfile
  output.close()


if __name__ == "__main__":
  atexit.register(cleanup)
  output.write("Time:," + formattedTime)
  signal.signal(signal.SIGTERM, lambda signum, stack_frame: exit(1))
  print "ctrl+c to quit"

  #send initialization command
  print "sending " + options.init
  ser.write(options.init + '\n')

  # throw this away
  while True:
    data = ser.readline()
    if data == "start_flag\r\n":
      break;

  while True:
    data = ser.readline()
    output.write(data) 
    output.flush()
