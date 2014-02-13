import socket
import json
import curses
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.connect(('192.168.42.1', 7000))

BLFR      = 0
BRFL      = 0
ROL       = 0
PIT       = 0

#chksum calculation
def chksum(str):
  c = 0
  for a in str:
    c = ((c + ord(a)) << 1) % 256
  return c
  
def makecommand(roll, pitch):
  sCom = '{"type":"cmp","r":%.4f,"p":%.4f}' % (roll, pitch) 
  return sCom
  
def sendcommand(sCom):
  sock.send(sCom);
  return sCom;

def keyevent(key):
  global BLFR
  global PIT
  global ROL
  global BRFL
  
  # Disarm if this button is pressed
  if key == ord("r"):
    ROL   = 0
    PIT   = 0
    BLFR  = 0
    BRFL  = 0
	
  # forward backward
  if key == ord("s") and PIT + 0.01 <= 10:
    PIT += 0.010;
  elif key == ord("w") and PIT - 0.01 >= -10:
    PIT -= 0.010;

  # strafe left right
  elif key == ord("d") and ROL + 0.01 <= 10:
    ROL += 0.010;
  elif key == ord("a") and ROL - 0.01 >= -10:
    ROL -= 0.010;
    
def main():
  stdscr = curses.initscr()
  curses.noecho()
  curses.cbreak()
  stdscr.keypad(1)
  #stdscr.nodelay(1)

  while 1:
      key = stdscr.getch()
      # Quit program
      if key == ord('x'):
        break
      else:
        keyevent(key)
        sCom = makecommand(ROL, PIT)
        print sendcommand(sCom)
        #time.sleep(0.100)

main()
