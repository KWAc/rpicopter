import socket
import json
import curses


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.connect(('192.168.42.1', 7000))

def make_pid(pit_rkp, pit_rki, pit_rimax, rol_rkp, rol_rki, rol_rimax, yaw_rkp, yaw_rki, yaw_rimax, pit_skp, rol_skp, yaw_skp):  
  sCom = '{"type":"pid_config","pit_rkp":%.4f,"pit_rki":%.4f,"pit_rimax":%.4f,"rol_rkp":%.4f,"rol_rki":%.4f,"rol_rimax":%.4f,"yaw_rkp":%.4f,"yaw_rki":%.4f,"yaw_rimax":%.4f,"pit_skp":%.4f,"rol_skp":%.4f,"yaw_skp":%.4f}' % (
          pit_rkp, pit_rki, pit_rimax, 
          rol_rkp, rol_rki, rol_rimax, 
          yaw_rkp, yaw_rki, yaw_rimax, 
          pit_skp, rol_skp, yaw_skp)

  return sCom
  
def sendcommand(sCom):
  sock.send(sCom)
  return sCom

f1 = make_pid(0.65, 0.35, 50.0,
             0.65, 0.35, 50.0,
             1.25, 0.25, 50.0,
             5.50, 5.50, 5.50)
 
f2 = make_pid(0.65, 0.30, 50.0,
             0.65, 0.30, 50.0,
             1.25, 0.25, 50.0,
             5.50, 5.50, 5.50)    
 
f3 = make_pid(0.65, 0.35, 50.0,
             0.65, 0.35, 50.0,
             1.25, 0.25, 50.0,
             5.00, 5.00, 5.00)  

f4 = make_pid(0.65, 0.35, 50.0,
             0.65, 0.35, 50.0,
             1.25, 0.20, 50.0,
             5.00, 5.00, 5.00)
                 
f5 = make_pid(0.60, 0.35, 50.0,
             0.60, 0.35, 50.0,
             1.25, 0.20, 50.0,
             5.00, 5.00, 5.00)
                     
f6 = make_pid(0.60, 0.30, 50.0,
             0.60, 0.30, 50.0,
             1.25, 0.20, 50.0,
             5.00, 5.00, 5.00)                     
                     
f7 = make_pid(0.55, 0.30, 50.0,
             0.55, 0.30, 50.0,
             1.25, 0.20, 50.0,
             5.00, 5.00, 5.00)                     
                     
f8 = make_pid(0.55, 0.25, 50.0,
             0.55, 0.25, 50.0,
             1.25, 0.20, 50.0,
             5.00, 5.00, 5.00)                     
                           
f9 = make_pid(0.5, 0.25, 50.0,
             0.5, 0.25, 50.0,
             1.25, 0.20, 50.0,
             5.00, 5.00, 5.00)                     
                     
f10 = make_pid(0.5, 0.20, 50.0,
             0.5, 0.20, 50.0,
             1.25, 0.20, 50.0,
             5.00, 5.00, 5.00)                     
                     

def main():
  stdscr = curses.initscr()
  curses.noecho()
  curses.cbreak()
  stdscr.keypad(1)

  while 1:
      key = stdscr.getch()
      # Quit program
      if key == ord('x'):
        break
      # Send one of the PID configurations
      if key == curses.KEY_F1:
        print "Send config 1"
        sendcommand(f1)
      if key == curses.KEY_F2:
        print "Send config 2"
        sendcommand(f2)        
      if key == curses.KEY_F3:
        print "Send config 3"
        sendcommand(f3)
      if key == curses.KEY_F4:
        print "Send config 4"
        sendcommand(f4)
      if key == curses.KEY_F5:
        print "Send config 5"
        sendcommand(f5)
      if key == curses.KEY_F6:
        print "Send config 6"
        sendcommand(f6)
      if key == curses.KEY_F7:
        print "Send config 7"
        sendcommand(f7)
      if key == curses.KEY_F8:
        print "Send config 8"
        sendcommand(f8)
      if key == curses.KEY_F9:
        print "Send config 9"
        sendcommand(f9)
      if key == curses.KEY_F10:
        print "Send config 10"
        sendcommand(f10)

main()