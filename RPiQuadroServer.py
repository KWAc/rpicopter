import socket
import json
import serial

import logging
from thread import start_new_thread, allocate_lock

from time import *

# Create a sensor log with date and time
logging.basicConfig(filename='/tmp/RPiQuadrocopter.log', level=logging.INFO, format='%(asctime)s %(message)s')

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 7000))

# Threading stuff
THREADS = 0
RECVR_STARTED = False
TRNSM_STARTED = False
THR_LOCK = allocate_lock()

# whether to use or not to use this timeout is the question?!
# rtscts=1 is necessary for raspbian due to a bug in the usb/serial driver ):
com_port  = '/dev/ttyACM0'
baud_rate = '115200'
try:
  ser = serial.Serial(com_port, baud_rate, timeout=0.1, writeTimeout=0.1, rtscts=1)
except serial.SerialException as e:
  logging.debug("Could not open serial port '{}': {}".format(com_port, e))
  
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
    bytes = ser.write(output)
  except serial.SerialTimeoutException as e:
    logging.error("Write timeout on serial port '{}': {}".format(com_port, e))
  # Flush input buffer, if there is still some unprocessed data left
  # Otherwise the APM 2.5 control boards stucks after some command
  ser.flush()       # Try to send old message
  ser.flushInput()  # Delete what is still inside the buffer

# These functions shall run in separate threads
# recv_thr() is used to catch sensor data
def recv_thr():
  # Cause it's not atomic operation
  global THREADS, RECVR_STARTED
  THR_LOCK.acquire()
  THREADS += 1
  RECVR_STARTED = True
  THR_LOCK.release()

  while True:
    THR_LOCK.acquire()
    while ser.inWaiting() > 0:
      try:
        # Remove newline character '\n'
        line = ser.readline().strip()
      except serial.SerialTimeoutException as e:
        logging.error("Read timeout on serial port '{}': {}".format(com_port, e))
      try:
        p = json.loads(line)
        if p['type'] == 'sens_comp' or p['type'] == 'sens_attitude' or p['type'] == 'sens_baro' or p['type'] == 'pid_config':
          logging.info(line)
      except (ValueError, KeyError, TypeError):
          # Print everything what is not valid json string to console
          print line
          #logging.debug("JSON format error: " + line)
    THR_LOCK.release()
      
  THR_LOCK.acquire()
  THREADS -= 1
  RECVR_STARTED = False
  THR_LOCK.release()

# trnm_thr() sends commands to APM2.5
def trnm_thr():
  # Cause it's not atomic operation
  global THREADS, TRNSM_STARTED
  THR_LOCK.acquire()
  THREADS += 1
  TRNSM_STARTED = True
  THR_LOCK.release()

  adr = msg = com = "" 
  while True:
    try:
      # Wait for UDP packet
      msg, adr = sock.recvfrom(1024)
    except socket.timeout:
      #logging.error("Read timeout on socket '{}': {}".format(adr, e))
      pass # Dunno, I think logging socket timeouts could become slow and unnecessary
    try:
      # parse line from socket
      p = json.loads(msg)
      # if control packet, send to APM
      if p['type'] == 'rc_input':
        com = "RC#%d,%d,%d,%d" % (p['roll'], p['pitch'], p['thr'], p['yaw'])
        THR_LOCK.acquire()
        send_data(com)
        THR_LOCK.release()
      if p['type'] == 'pid_config':
        com = "PID#%.4f,%.4f,%.4f;%.4f,%.4f,%.4f;%.4f,%.4f,%.4f;%.4f,%.4f,%.4f" % (
          p['pit_rkp'], p['pit_rki'], p['pit_rimax'], 
          p['rol_rkp'], p['rol_rki'], p['rol_rimax'], 
          p['yaw_rkp'], p['yaw_rki'], p['yaw_rimax'], 
          p['pit_skp'], p['rol_skp'], p['yaw_skp'] )
        THR_LOCK.acquire()
        send_data(com)
        THR_LOCK.release()
    except (ValueError, KeyError, TypeError):
      print com.strip()
      #logging.debug("JSON format error: " + com)

  # Cause it's not atomic operation
  THR_LOCK.acquire()
  THREADS -= 1
  TRNSM_STARTED = False
  THR_LOCK.release()
  
#main loop for sending and receiving
def main():
  # Nice to see how python supports function pointer in such an easy way 
  start_new_thread(recv_thr, ())
  start_new_thread(trnm_thr, ())

  # Wait if no thread was started so far and ..
  while not RECVR_STARTED or not TRNSM_STARTED:
    pass
  # if threads was started, because they close if main programs end
  while THREADS > 0:
    pass
  
  ser.close()

main()
