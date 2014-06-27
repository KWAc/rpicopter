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
import os
import time

# Create a sensor log with date and time
layout = '%(asctime)s - %(levelname)s - %(message)s'
logging.basicConfig(filename='/tmp/RPiQuadrocopter.log', level=logging.INFO, format=layout)

# Socket for WiFi data transport
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.bind(('0.0.0.0', 7000))
udp_client  = None
pySerial    = None

# Thread lock for multi threading
THR_LOCK = threading.Lock()

def init_serial(device_count = 10):
  baudrate = '115200'
  for counter in range (device_count):
    port = open_port('/dev/ttyACM%d' % (counter), baudrate)
    if port:
      return port
    port = open_port('/dev/ttyUSB%d' % (counter), baudrate)
    if port:
      return port
  return None

def open_port(portname, baudrate):
  try:
    # Set timeouts to 4 ms. 3.5 ms is the measured bias when long msg get cut!
    return serial.Serial(portname, baudrate, timeout=0.004, writeTimeout=0.004)
  except serial.SerialException as e:
    logging.debug("Could not open serial port: {}".format(portname, e))
    print ("Could not open serial port: {}".format(portname, e))
  return None

def chksum(line):
  c = 0
  for a in line:
    c = ((c + ord(a)) << 1) % 256
  return c

def ser_write(type, line):
  chk = chksum(line)
  output = "%s%s*%x\r\n" % (type, line, chk)                            # Concatenate msg and chksum
  
  if pySerial is not None:
    THR_LOCK.acquire()
    try:
      bytes = pySerial.write(output)
    except serial.SerialTimeoutException as e:
      logging.error("Write timeout on serial port")
    except serial.SerialException as e:
      logging.error("Write exception serial port")
    finally:
      pySerial.flushInput()                                             # Workaround: free write buffer (otherwise the Arduino board hangs)
    THR_LOCK.release()
    
def udp_write(msg, adress):
  if adress is not None:
    bytes = udp_sock.sendto(msg, adress)

def make_command(type, p):
  # remote control is about controlling the model (thrust and attitude)
  if type == 'rc':
    com = "%d,%d,%d,%d" % (p['r'], p['p'], p['t'], p['y'])
    ser_write("RC#", com)

  # Add a waypoint
  if type == 'uav':
    com = "%d,%d,%d,%d" % (p['lat_d'], p['lon_d'], p['alt_m'], p['flag_t'] )
    ser_write("UAV#", com)

  # PID config is about to change the sensitivity of the model to changes in attitude
  if type == 'pid':
    com = "%.2f,%.2f,%.4f,%.2f;%.2f,%.2f,%.4f,%.2f;%.2f,%.2f,%.4f,%.2f;%.2f,%.2f,%.4f,%.2f;%.2f,%.2f,%.4f,%.2f;%.2f,%.2f,%.2f,%.2f,%.2f" % (
      p['p_rkp'], p['p_rki'], p['p_rkd'], p['p_rimax'],
      p['r_rkp'], p['r_rki'], p['r_rkd'], p['r_rimax'],
      p['y_rkp'], p['y_rki'], p['y_rkd'], p['y_rimax'],
      p['t_rkp'], p['t_rki'], p['t_rkd'], p['t_rimax'],
      p['a_rkp'], p['a_rki'], p['a_rkd'], p['a_rimax'],
      p['p_skp'], p['r_skp'], p['y_skp'], p['t_skp'], p['a_skp'] )
    ser_write("PID#", com)

  # This section is about correcting drifts while model is flying (e.g. due to imbalances of the model)
  if type == 'cmp':
    com = "%.2f,%.2f" % (p['r'], p['p'])
    ser_write("CMP#", com)

  # With this section you may start the calibration of the gyro again
  if type == 'gyr':
    com = "%d" % (p['cal'])
    ser_write("GYR#", com)

  # User interactant for gyrometer calibration
  if type == 'user_interactant':
    bytes = pySerial.write("x") # write a char into the serial device
    pySerial.flushInput()
    
  # Ping service for calculating the latency of the connection
  if type == 'ping':
    com = '{"type":"pong","v":%d}' % (p['v'])
    udp_write(com, udp_client)
    
def recv_thr():                                                         # recv_thr() is used to catch sensor data
  global udp_client
  ser_msg = None

  while pySerial is not None:
    if not pySerial.readable() or not pySerial.inWaiting() > 0:
      continue
      
    try:
      THR_LOCK.acquire()
      ser_msg = pySerial.readline().strip()                           # Remove newline character '\n'
      THR_LOCK.release()
    except serial.SerialTimeoutException as e:
      logging.error("Read timeout on serial port")
    except serial.SerialException as e:
      logging.error("Read exception on serial port")
    else:
      try:
        p = json.loads(ser_msg)
      except (ValueError, KeyError, TypeError):
        #print ("JSON format error: %s" % ser_msg)                       # Print everything what is not a valid JSON string to console
        ser_msg = '{"type":"NOJSON","data":"%s"}' % ser_msg
      finally:
        udp_write(ser_msg, udp_client)

def trnm_thr():                                                         # trnm_thr() sends commands to Arduino
  global udp_client

  while pySerial is not None:
    if not pySerial.writable():
      continue
      
    try:
      udp_msg, udp_client = udp_sock.recvfrom(512)                      # Wait for UDP packet from ground station
    except socket.timeout:
      logging.error("Write timeout on socket")                          # Log the problem
    else:
      try:
        p = json.loads(udp_msg)                                         # parse JSON string from socket
      except (ValueError, KeyError, TypeError):
        logging.debug("JSON format error: " + udp_msg.strip() )
      else:
        make_command(p['type'], p)

def main():
  recv=threading.Thread(target=recv_thr)
  trnm=threading.Thread(target=trnm_thr)
  recv.start()
  trnm.start()

pySerial = init_serial()
if pySerial is None:
  print ("Could not open any serial port. Exit script.")
  sys.exit(0)
main()
