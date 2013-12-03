import socket
import json


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.connect(('192.168.42.1', 7000))

def makepid(pit_rkp, pit_rki, pit_rimax, rol_rkp, rol_rki, rol_rimax, yaw_rkp, yaw_rki, yaw_rimax, pit_skp, rol_skp, yaw_skp):  
  sCom = '{"type":"pid_config","pit_rkp":%.4f,"pit_rki":%.4f,"pit_rimax":%.4f,"rol_rkp":%.4f,"rol_rki":%.4f,"rol_rimax":%.4f,"yaw_rkp":%.4f,"yaw_rki":%.4f,"yaw_rimax":%.4f,"pit_skp":%.4f,"rol_skp":%.4f,"yaw_skp":%.4f}' % (
          pit_rkp, pit_rki, pit_rimax, 
          rol_rkp, rol_rki, rol_rimax, 
          yaw_rkp, yaw_rki, yaw_rimax, 
          pit_skp, rol_skp, yaw_skp)

  return sCom
  
def sendcommand(sCom):
  sock.send(sCom)
  return sCom
  

sCom = makepid(0.35, 0.15, 50,
               0.35, 0.15, 50,
               0.35, 0.15, 50,
               4.5,  4.5,  4.5)

print "End process with: " + sendcommand(sCom)