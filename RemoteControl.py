import socket
import json
import curses
import time

# Global ranges for controlling the quadrocopter
# Units in degrees
YAW_MIN   = -20
YAW_MAX   = +20
PIT_MIN   = -10
PIT_MAX   = +10
ROL_MIN   = -10
ROL_MAX   = +10
THR_MIN   = 1100
THR_MAX   = 1900
THR_80P   = 0.5 * (THR_MAX - THR_MIN) + THR_MIN
# Global variables for controlling the quadrocopter
ROL       = (ROL_MAX - ROL_MIN) / 2 + ROL_MIN
PIT       = (PIT_MAX - PIT_MIN) / 2 + PIT_MIN
YAW       = (ROL_MAX - ROL_MIN) / 2 + ROL_MIN
THR       = THR_MIN

key_pressed = 0

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.connect(('192.168.42.1', 7000))


#chksum calculation
def chksum(str):
  c = 0
  for a in str:
    c = ((c + ord(a)) << 1) % 256
  return c

def gyro_calibration():
  sCom = '{"type":"gyr","cal":%d"}' % (True)
  return sCom
  
def makecommand(roll, pitch, throttle, yaw):
  sCom = '{"type":"rc","r":%d,"p":%d,"t":%d,"y":%d}' % (roll, pitch, throttle, yaw) 
  return sCom
  
def sendcommand(sCom):
  sock.send(sCom);
  return sCom;

def reset_stats(key):
  global THR
  global PIT
  global ROL
  global YAW
  global key_pressed
  
  # Reset the settings stepwise
  if key == -1 and time.time() - key_pressed > 0.35:
    key_pressed = time.time()
    if PIT > 0:
      PIT -= 1
    if ROL > 0:
      ROL -= 1
    if YAW > 0:
      YAW -= 1
    
    if PIT < 0:
      PIT += 1
    if ROL < 0:
      ROL += 1
    if YAW < 0:
      YAW += 1
  
def keyevent(key):
  global THR
  global PIT
  global ROL
  global YAW
  global key_pressed
  
  
  # higher lower thrust
  #if key == curses.KEY_UP and THR < 0.8*(THR_MAX - THR_MIN)+THR_MIN: # Never upregulate till maximum
  #if key == curses.KEY_UP and THR < THR_80P:
  if key == curses.KEY_UP and THR < THR_80P:
    key_pressed = time.time()
    THR += 5
  if key == curses.KEY_DOWN and THR > THR_MIN:
    key_pressed = time.time()
    THR -= 5
  
  # Disarm if this button is pressed
  if key == ord("r"):
    key_pressed = time.time()
    ROL = (ROL_MAX - ROL_MIN) / 2 + ROL_MIN
    PIT = (PIT_MAX - PIT_MIN) / 2 + PIT_MIN
    YAW = (ROL_MAX - ROL_MIN) / 2 + ROL_MIN
    THR = THR_MIN
	
  # forward backward
  if key == ord("s") and PIT + PIT_MAX/10 <= PIT_MAX:
    key_pressed = time.time()
    PIT += PIT_MAX/10;
  elif key == ord("w") and PIT + PIT_MIN/10 >= PIT_MIN:
    key_pressed = time.time()
    PIT += PIT_MIN/10;

  # strafe left right
  elif key == ord("d") and ROL + ROL_MAX/10 <= ROL_MAX:
    key_pressed = time.time()
    ROL += ROL_MAX/10;
  elif key == ord("a") and ROL + ROL_MIN/10 >= ROL_MIN:
    key_pressed = time.time()
    ROL += ROL_MIN/10;

  # Turn left right
  elif key == ord("q") and YAW + YAW_MAX/10 <= YAW_MAX:
    key_pressed = time.time()
    YAW += YAW_MAX/10;
  elif key == ord("e") and YAW + YAW_MIN/10 >= YAW_MIN:
    key_pressed = time.time()
    YAW += YAW_MIN/10;
    
  reset_stats(key)
    
def main():
  stdscr = curses.initscr()
  curses.noecho()
  curses.cbreak()
  stdscr.keypad(1)
  stdscr.nodelay(1)

  while 1:
      key = stdscr.getch()
      # Quit program
      if key == ord('x'):
        sCom = makecommand(0, 0, THR_MIN, 0)
        print "End process with: " + sendcommand(sCom)
        break
      # Start gyro calibration
      if key == ord("c"):
        sCom = gyro_calibration()
        sendcommand(sCom)
        time.sleep(2.0)
      else:
        keyevent(key)
        sCom = makecommand(ROL, PIT, THR, YAW)
        print sendcommand(sCom)
        time.sleep(0.010)

main()
