# Make this Python 2.7 script compatible to Python 3 standard
from __future__ import print_function
# For remote control
import socket
import json
import serial
# For sensor readout
import logging
import threading
# For system specific functions
import sys

from time import *

# Create a sensor log with date and time
layout = '%(asctime)s - %(levelname)s - %(message)s'
logging.basicConfig(filename='/tmp/RPiQuadrocopter.log', level=logging.INFO, format=layout)

# Socket for WiFi data transport
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.bind(('0.0.0.0', 7000))
client_adr = ""

# Thread lock for multi threading
THR_LOCK = threading.Lock()

#pySerial
pySerial = 0

def init_serial(device_count = 10):
  counter   = 0
  baud_rate = '115200'
  while counter < device_count:
    com_port  = '/dev/ttyACM%d' % (counter)
    try:
      # rtscts=1 is necessary for raspbian due to a bug in the usb/serial driver
      pySerial = serial.Serial(com_port, baud_rate, timeout=0.1, writeTimeout=0.1, rtscts=1)
    except serial.SerialException as e:
      logging.debug("Could not open serial port: {}".format(com_port, e))
      print ("Could not open serial port: {}".format(com_port, e))
      com_port = '/dev/ttyUSB%d' % (counter)
      try:
        # rtscts=1 is necessary for raspbian due to a bug in the usb/serial driver
        pySerial = serial.Serial(com_port, baud_rate, timeout=0.1, writeTimeout=0.1, rtscts=1)
      except serial.SerialException as e:
        logging.debug("Could not open serial port: {}".format(com_port, e))
        print ("Could not open serial port: {}".format(com_port, e))
        if counter == device_count-1:
          return False, pySerial
        counter += 1
      else: 
        return True, pySerial
    else: 
      return True, pySerial
  
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

# These functions shall run in separate threads
# recv_thr() is used to catch sensor data
def recv_thr():
  global client_adr
  ser_line = ""
  while True:
    # Lock while data in queue to get red
    THR_LOCK.acquire()
    while pySerial.inWaiting() > 0:
      try:
        # Remove newline character '\n'
        ser_line = pySerial.readline().strip()
      except serial.SerialTimeoutException as e:
        logging.error("Read timeout on serial port '{}': {}".format(com_port, e))
      else:
        try:
          p = json.loads(ser_line)
        except (ValueError, KeyError, TypeError):
          # Print everything what is not valid json string to console
          print ("JSON format error: %s" % ser_line)
          #logging.debug("JSON format error: " + ser_line)
        else: 
          logging.info(ser_line)
          if client_adr != "":
            bytes = udp_sock.sendto(ser_line, client_adr)
            #print("udp_sock send %d bytes" % bytes)
        
    THR_LOCK.release()

# trnm_thr() sends commands to APM2.5
def trnm_thr():
  global client_adr
  msg = ""
  com = "" 
  while True:
    try:
      # Wait for UDP packet
      msg, client_adr = udp_sock.recvfrom(256)
      
    except socket.timeout:
      #logging.error("Read timeout on socket '{}': {}".format(adr, e))
      pass # Dunno, I think logging socket timeouts could become slow and unnecessary
     
    try:
      # parse JSON string from socket
      p = json.loads(msg)

      # remote control is about controlling the model (thrust and attitude)
      if p['type'] == 'rc':
        com = "RC#%d,%d,%d,%d" % (p['r'], p['p'], p['t'], p['y'])
        THR_LOCK.acquire()
        send_data(com)
        THR_LOCK.release()
        
      # PID config is about to change the sensitivity of the model to changes in attitude
      if p['type'] == 'pid':
        com = "PID#%.2f,%.2f,%.2f;%.2f,%.2f,%.2f;%.2f,%.2f,%.2f;%.2f,%.2f,%.2f" % (
          p['pit_rkp'], p['pit_rki'], p['pit_rimax'], 
          p['rol_rkp'], p['rol_rki'], p['rol_rimax'], 
          p['yaw_rkp'], p['yaw_rki'], p['yaw_rimax'], 
          p['pit_skp'], p['rol_skp'], p['yaw_skp'] )
        THR_LOCK.acquire()
        send_data(com)
        THR_LOCK.release()
        
      # This section is about correcting drifts while model is flying (e.g. due to imbalances of the model)
      if p['type'] == 'cmp':
        com = "CMP#%.2f,%.2f" % (p['r'], p['p'])
        THR_LOCK.acquire()
        send_data(com)
        THR_LOCK.release()
        
      # With this section you may start the calibration of the gyro again
      if p['type'] == 'gyr':
        com = "GYR#%d" % (p['cal'])
        THR_LOCK.acquire()
        send_data(com)
        THR_LOCK.release()
        
    except (ValueError, KeyError, TypeError):
      print (com.strip() )
      #logging.debug("JSON format error: " + com.strip() )
  
# Main program for sending and receiving
# Working with two separate threads
def main():
  # Start threads for receiving and transmitting
  recv=threading.Thread(target=recv_thr)
  trnm=threading.Thread(target=trnm_thr)
  recv.start()
  trnm.start()

# Start Program
bInitialized, pySerial = init_serial()
if not bInitialized:
  print ("Could not open any serial port. Exit script.")
  sys.exit()
main()
