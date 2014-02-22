# Make this Python 2.7 script compatible to Python 3 standard
from __future__ import print_function
# For remote control
import serial
import time

com_port = 3
baud_rate = '9600'
#baud_rate = '115200'
pySerial = serial.Serial(com_port, baud_rate)
 
 
#chksum calculation
def chksum(line):
  c = 0
  for a in line:
    c = ((c + ord(a)) << 1) % 256
  return c
  
def send_data(line):
  # calc checksum
  chk = chksum(line)
  # concatenate msg and chksum
  output = "%s*%x\r\n" % (line, chk)
  try:
    bytes = pySerial.write(output)
  except serial.SerialTimeoutException as e:
    logging.error("Write timeout on serial port '{}': {}".format(com_port, e))
  finally:
    # Flush input buffer, if there is still some unprocessed data left
    # Otherwise the APM 2.5 control boards stucks after some command
    pySerial.flushInput()  # Delete what is still inside the buffer
 
# Main program for sending and receiving
# Working with two separate threads
def main():
  while(True):
    time.sleep(0.01)
    #line = "RC#0,0,1200,0"
    #send_data(line);
    
    line = chr(2) + chr(50) + chr(127) + chr(127) + chr(128) + chr(0);
    print (chksum(line))
    line += chr(chksum(line))
    line += chr(254)
    pySerial.write(line)
    
    pySerial.flushInput()
#    while pySerial.inWaiting() > 0:
#      print (pySerial.read() )

# Start Program
main()
