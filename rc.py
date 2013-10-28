import socket
import json
import curses
import time

# Global ranges for controlling the quadrocopter
# Units in degrees
YAW_MIN   = -180
YAW_MAX   = +180
PIT_MIN   = -45
PIT_MAX   = +45
ROL_MIN   = -45
ROL_MAX   = +45
THR_MIN   = 1100
THR_MAX   = 1900
THR_80P   = 0.8 * (THR_MAX - THR_MIN) + THR_MIN
# Global variables for controlling the quadrocopter
ROL       = (ROL_MAX - ROL_MIN) / 2 + ROL_MIN
PIT       = (PIT_MAX - PIT_MIN) / 2 + PIT_MIN
YAW       = (ROL_MAX - ROL_MIN) / 2 + ROL_MIN
THR       = THR_MIN

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#sock.connect(('192.168.42.1', 7000))
sock.connect(('192.168.1.5', 7000))

#chksum calculation
def chksum(str):
  c = 0
  for a in str:
    c = ((c + ord(a)) << 1) % 256
  return c

def makecommand(roll, pitch, throttle, yaw):
  sCom = '{"type":"rcinput","roll":%d,"pitch":%d,"thr":%d,"yaw":%d}' % (roll, pitch, throttle, yaw)
  
  p = json.loads(sCom)
  str = "%d,%d,%d,%d" % (p['roll'], -p['pitch'], p['thr'], p['yaw'])
  #calc checksum
  chk = chksum(str)
  #concatenate msg and chksum
  output = "%s*%x\r\n" % (str, chk)
  print "Send to control board: " + output
  
  return sCom

def sendcommand(sCom):
  sAdd = "*%d" % (chksum(sCom) )
  sCom = sCom + sAdd
  sock.send(sCom);
  return sCom;

def keyevent(key):
  global THR
  global PIT
  global ROL
  global YAW
  
  # Reset the settings stepwise
  if PIT > (PIT_MAX - PIT_MIN) / 2 + PIT_MIN:
	PIT -= 2.5
  if ROL > (ROL_MAX - ROL_MIN) / 2 + ROL_MIN:
	ROL -= 2.5
  if YAW > (YAW_MAX - YAW_MIN) / 2 + YAW_MIN:
	YAW -= 20
	
  if PIT < (PIT_MAX - PIT_MIN) / 2 + PIT_MIN:
	PIT += 2.5
  if ROL < (ROL_MAX - ROL_MIN) / 2 + ROL_MIN:
	ROL += 2.5
  if YAW < (YAW_MAX - YAW_MIN) / 2 + YAW_MIN:
	YAW += 20
  
  # higher lower thrust
  #if key == curses.KEY_UP and THR < 0.8*(THR_MAX - THR_MIN)+THR_MIN: # Never upregulate till maximum
  #if key == curses.KEY_UP and THR < THR_80P:
  if key == curses.KEY_UP and THR < THR_MAX:
    THR += 12.5
  if key == curses.KEY_DOWN and THR > THR_MIN:
    THR -= 12.5

  # Disarm if this button is pressed
  if key == ord("r"):
	ROL = (ROL_MAX - ROL_MIN) / 2 + ROL_MIN
	PIT = (PIT_MAX - PIT_MIN) / 2 + PIT_MIN
	YAW = (ROL_MAX - ROL_MIN) / 2 + ROL_MIN
	THR = THR_MIN
	
  # forward backward
  if key == ord("w") and PIT < PIT_MAX:
    PIT = PIT_MAX;
  if key == ord("s") and PIT > PIT_MIN:
    PIT = PIT_MIN;

  # strafe left right
  if key == ord("a") and ROL < ROL_MAX:
    ROL = ROL_MAX;
  if key == ord("d") and ROL > ROL_MIN:
    ROL = ROL_MIN;

  # Turn left right
  if key == ord("q") and YAW < YAW_MAX:
    YAW = YAW_MAX;
  if key == ord("e") and YAW > YAW_MIN:
    YAW = YAW_MIN;

def main():
  stdscr = curses.initscr()
  curses.noecho()
  curses.cbreak()
  stdscr.keypad(1)
  stdscr.nodelay(1)

  while 1:
      key = stdscr.getch()
      if key == ord('x'):
        sCom = makecommand(0, 0, 0, THR_MIN)
        print sendcommand(sCom)
        time.sleep(1)
        break
      else:
        keyevent(key)
        sCom = makecommand(ROL, PIT, THR, YAW)
        print sendcommand(sCom)
        time.sleep(0.025)

main()
