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
          return False, 0
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

def send_data(type, line):
  # calc checksum
  chk = chksum(line)
  # concatenate msg and chksum
  output = "%s%s*%x\r\n" % (type, line, chk)
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
  try:
    while pySerial.readable():
      # Lock while data in queue to get red
      THR_LOCK.acquire()
      while pySerial.inWaiting() > 0:
        # Remove newline character '\n'
        ser_line = pySerial.readline().strip()
        try:
          p = json.loads(ser_line)
        except (ValueError, KeyError, TypeError):
          # Print everything what is not a valid JSON string to console
          print ("JSON format error: %s" % ser_line)
          # Send the string to the client after is was flagged
          nojson_line = '{"type":"NOJSON","data":"%s"}' % ser_line
          if client_adr != "":
            bytes = udp_sock.sendto(nojson_line, client_adr)
        else:
          logging.info(ser_line)
          if client_adr != "":
            bytes = udp_sock.sendto(ser_line, client_adr)
      THR_LOCK.release()
  # Terminate process (makes restarting in the init.d part possible)
  except:
    os.kill(os.getpid(), 15)

# trnm_thr() sends commands to APM2.5
def trnm_thr():
  global client_adr
  msg = ""
  try:
    while pySerial.writable():
      try:
        # Wait for UDP packet from ground station
        msg, client_adr = udp_sock.recvfrom(512)
      except socket.timeout:
        # Log the problem
        logging.error("Read timeout on socket '{}': {}".format(adr, e))
      else:
        try:
          # parse JSON string from socket
          p = json.loads(msg)
        except (ValueError, KeyError, TypeError):
          logging.debug("JSON format error: " + msg.strip() )
        else:
          # remote control is about controlling the model (thrust and attitude)
          if p['type'] == 'rc':
            com = "%d,%d,%d,%d" % (p['r'], p['p'], p['t'], p['y'])
            THR_LOCK.acquire()
            send_data("RC#", com)
            THR_LOCK.release()

          # Add a waypoint
          if p['type'] == 'uav':
            com = "%d,%d,%d,%d" % (p['lat_d'], p['lon_d'], p['alt_m'], p['flag_t'] )
            THR_LOCK.acquire()
            send_data("UAV#", com)
            THR_LOCK.release()

          # PID config is about to change the sensitivity of the model to changes in attitude
          if p['type'] == 'pid':
            com = "%.2f,%.2f,%.4f,%.2f;%.2f,%.2f,%.4f,%.2f;%.2f,%.2f,%.4f,%.2f;%.2f,%.2f,%.4f,%.2f;%.2f,%.2f,%.4f,%.2f;%.2f,%.2f,%.2f,%.2f,%.2f" % (
              p['p_rkp'], p['p_rki'], p['p_rkd'], p['p_rimax'],
              p['r_rkp'], p['r_rki'], p['r_rkd'], p['r_rimax'],
              p['y_rkp'], p['y_rki'], p['y_rkd'], p['y_rimax'],
              p['t_rkp'], p['t_rki'], p['t_rkd'], p['t_rimax'],
              p['a_rkp'], p['a_rki'], p['a_rkd'], p['a_rimax'],
              p['p_skp'], p['r_skp'], p['y_skp'], p['t_skp'], p['a_skp'] )
            THR_LOCK.acquire()
            send_data("PID#", com)
            THR_LOCK.release()

          # This section is about correcting drifts while model is flying (e.g. due to imbalances of the model)
          if p['type'] == 'cmp':
            com = "%.2f,%.2f" % (p['r'], p['p'])
            THR_LOCK.acquire()
            send_data("CMP#", com)
            THR_LOCK.release()

          # With this section you may start the calibration of the gyro again
          if p['type'] == 'gyr':
            com = "%d" % (p['cal'])
            THR_LOCK.acquire()
            send_data("GYR#", com)
            THR_LOCK.release()

          # Ping service for calculating the latency of the connection
          if p['type'] == 'ping':
            com = '{"type":"pong","v":%d}' % (p['v'])
            if client_adr != "":
              bytes = udp_sock.sendto(com, client_adr)
              
          # User interactant for gyrometer calibration
          if p['type'] == 'user_interactant':
            THR_LOCK.acquire()
            bytes = pySerial.write("x") # write a char into the serial device
            pySerial.flushInput()
            THR_LOCK.release()

  # Terminate process (makes restarting in the init.d part possible)
  except:
    os.kill(os.getpid(), 15)

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
  sys.exit(0)
main()
