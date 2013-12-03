import json
from time import *


#chksum calculation
def chksum(str):
  c = 0
  for a in str:
    c = ((c + ord(a)) << 1) % 256
  return c


str = "PID#%.4f,%.4f,%.4f;%.4f,%.4f,%.4f;%.4f,%.4f,%.4f;%.4f,%.4f,%.4f" % (
0, 1, 2, 
5, 50, 0, 
8, 9, 10,  
0.5, 5.0, 10,  )
# calc checksum
chk = chksum(str)

# concatenate msg and chksum
output = "%s*%x\r\n" % (str, chk)
print "Send to control board: " + output

