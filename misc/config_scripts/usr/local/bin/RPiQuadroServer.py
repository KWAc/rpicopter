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
udp_server  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_server.bind(('0.0.0.0', 7000))

udp_clients = set()
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

def ser_write(line):
  if pySerial is not None:
    THR_LOCK.acquire()
    try:
      bytes = pySerial.write(line)
    except serial.SerialTimeoutException:
      logging.error("Write time-out on serial port")
    except serial.SerialException:
      logging.error("Write exception serial port")
    finally:
      pySerial.flushInput()                                             # Workaround: free write buffer (otherwise the Arduino board hangs)
    THR_LOCK.release()

def udp_write(msg, clients):
  for client in clients:
    bytes = udp_server.sendto(msg, client)

def send_command(type, line):
  chk = chksum(line)
  output = "%s%s*%x\r\n" % (type, line, chk)                            # Concatenate msg and chksum
  ser_write(output)

def make_command(p):
  default = ""
  type = p.get('t', default)

  # remote control is about controlling the model (thrust and attitude)
  if type == 'rc':
    try:
      com = "%d,%d,%d,%d" % (p['r'], p['p'], p['f'], p['y'])
    except KeyError, ValueError:
      logging.error("KeyError in make_command()")
    else:
      send_command("RC#", com)

  # Add a waypoint
  if type == 'uav':
    try:
      com = "%d,%d,%d,%d" % (p['lat_d'], p['lon_d'], p['alt_m'], p['flag_t'])
    except KeyError, ValueError:
      logging.error("KeyError in make_command()")
    else:
      send_command("UAV#", com)

  # PID config is about to change the sensitivity of the model to changes in attitude
  if type == 'pid':
    try:
      com = "%.2f,%.2f,%.4f,%.2f;%.2f,%.2f,%.4f,%.2f;%.2f,%.2f,%.4f,%.2f;%.2f,%.2f,%.4f,%.2f;%.2f,%.2f,%.4f,%.2f;%.2f,%.2f,%.2f,%.2f,%.2f" % (
        p['p_rkp'], p['p_rki'], p['p_rkd'], p['p_rimax'],
        p['r_rkp'], p['r_rki'], p['r_rkd'], p['r_rimax'],
        p['y_rkp'], p['y_rki'], p['y_rkd'], p['y_rimax'],
        p['t_rkp'], p['t_rki'], p['t_rkd'], p['t_rimax'],
        p['a_rkp'], p['a_rki'], p['a_rkd'], p['a_rimax'],
        p['p_skp'], p['r_skp'], p['y_skp'], p['t_skp'], p['a_skp'])
    except KeyError, ValueError:
      logging.error("KeyError in make_command()")
    else:
      send_command("PID#", com)

  # This section is about correcting drifts while model is flying (e.g. due to imbalances of the model)
  if type == 'cmp':
    try:
      com = "%.2f,%.2f" % (p['r'], p['p'])
    except KeyError, ValueError:
      logging.error("KeyError in make_command()")
    else:
      send_command("CMP#", com)

  # This section is about correcting the compass
  if type == 'comp':
    try:
      com = "%d,%d,%.4f,%.4f,%.4f" % (p['t'], p['n'], p['x'], p['y'], p['z'])
    except KeyError, ValueError:
      logging.error("KeyError in make_command()")
    else:
      send_command("COMP#", com)
      
  # With this section you may start the calibration of the gyro again
  if type == 'gyr':
    try:
      com = "%d" % (p['cal'])
    except KeyError, ValueError:
      logging.error("KeyError in make_command()")
    else:
      send_command("GYR#", com)

  # User interactant for gyrometer calibration
  if type == 'user_interactant':
    ser_write("x")

  # Ping service for calculating the latency of the connection
  if type == 'ping':
    try:
      com = '{"t":"pong","v":%d}' % (p['v'])
    except KeyError, ValueError:
      logging.error("KeyError in make_command()")
    else:
      udp_write(com, udp_clients)

def recv_thr():                                                         # recv_thr() is used to catch sensor data
  global udp_clients
  ser_msg = None

  while pySerial is not None:
    if not pySerial.readable() or not pySerial.inWaiting() > 0:
      continue

    try:
      THR_LOCK.acquire()
      ser_msg = pySerial.readline().strip()                             # Remove newline character '\n'
      THR_LOCK.release()
    except serial.SerialTimeoutException:
      logging.error("Read time-out on serial port")
      continue
    except serial.SerialException:
      logging.error("Read exception on serial port")
      continue

    try:
      p = json.loads(ser_msg)
    except (ValueError, KeyError, TypeError):
      #print ("JSON format error: %s" % ser_msg)                        # Print everything what is not a valid JSON string to console
      ser_msg = '{"t":"NOJSON","data":"%s"}' % ser_msg
    finally:
      udp_write(ser_msg, udp_clients)

def trnm_thr():                                                         # trnm_thr() sends commands to Arduino
  global udp_clients
  udp_client = None
  udp_msg    = None

  #udp_socket.settimeout(.02)
  while pySerial is not None:
    if not pySerial.writable():
      continue

    try:
      udp_msg, udp_client = udp_server.recvfrom(512)                    # Wait for UDP packet from ground station
    except socket.timeout:
      logging.error("Write timeout on socket")                          # Log the problem
      continue
    
    if udp_client is not None: 
      udp_clients.add(udp_client)                                       # Add new client to client list
    if not udp_msg: 
      continue                                                          # If message is empty continue without parsing
    
    try:
      p = json.loads(udp_msg)                                           # parse JSON string from socket
    except (ValueError, KeyError, TypeError):
      logging.debug("JSON format error: " + udp_msg.strip() )
    else:
      make_command(p)

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
