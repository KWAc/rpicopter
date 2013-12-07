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
  

medPID = makepid(0.65, 0.35, 50.0,
                 0.65, 0.35, 50.0,
                 1.25, 0.20, 50.0,
                 5.00, 5.00, 5.00)
                 
senPID = makepid(0.55, 0.35, 50.0,
                 0.55, 0.35, 50.0,
                 1.00, 0.40, 50.0,
                 5.50, 5.50, 5.50)

print "End process with: " + sendcommand(medPID)