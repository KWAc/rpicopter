import socket
import json
import serial
from time import *

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 7000))
# whether to use or not to use this timeout is the question?!
# rtscts=1 is necessary for raspbian due to a bug in the usb/serial driver ):
ser = serial.Serial('/dev/ttyACM0', '38400', writeTimeout=0.1, rtscts=1)

#chksum calculation
def chksum(str):
  c = 0
  for a in str:
    c = ((c + ord(a)) << 1) % 256
  return c

#main loop for receiving a json string
#The checksum will get validated on every important step
def main():
  while True:
    # Wait for UDP packet
    data, addr = sock.recvfrom(128)
    fchksum = int(data[data.find("*")+1 : len(data)])
    data = data[: data.find("*")]

    #print data

    # Do nothing if checksum of incoming message is wrong
    if chksum(data) != fchksum:
      print "Wrong checksum! Received: %d, calculated: %d" % (fchksum, chksum(data) )
      continue
    else:
      # parse it
      p = json.loads(data)

      # if control packet, send to ardupilot
      if p['type'] == 'rcinput':
        str = "%d,%d,%d,%d" % (p['roll'], -p['pitch'], p['thr'], p['yaw'])
        # calc checksum
        chk = chksum(str)

        # concatenate msg and chksum
        output = "%s*%x\r\n" % (str, chk)
        #print "Send to control board: " + output

        ser.write(output)
        # flushInput() seems to be extremely important
        # otherwise the APM 2.5 control boards stucks after some command
        ser.flushInput()

main()
