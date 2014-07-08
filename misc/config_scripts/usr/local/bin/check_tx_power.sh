#!/bin/sh

CURR_TX=$(iwconfig wlan0 | head -1 | awk '{print $5}' | cut -d '=' -f 2)

while [ 1 ]
do
  if [ "$CURR_TX" = "30" ]; then
    echo "Do not change Tx power"
  else
    echo "Change Tx power to 30 dbi"
    /sbin/iw reg set BO
    sleep 0.5
    /sbin/iwconfig wlan0 txpower 30
  fi
  sleep 60
done 
