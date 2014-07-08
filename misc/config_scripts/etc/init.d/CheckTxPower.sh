#! /bin/sh
# /etc/init.d/CheckTxPower
#

WORK_DIR="/var/lib/CheckTxPower"
DAEMON="/bin/sh"
ARGS="/usr/local/bin/check_Tx_power.sh"
PIDFILE="/var/run/CheckTxPower.pid"
USER="root"

. /lib/lsb/init-functions

do_start() {
  log_daemon_msg "Starting system $DAEMON $ARGS daemon"
  mkdir -p "$WORK_DIR"
  /sbin/start-stop-daemon --start --pidfile $PIDFILE \
    --user $USER --group $USER \
    -b --make-pidfile \
    --chuid $USER \
    --exec $DAEMON $ARGS
  log_end_msg $?
}

do_stop() {
  log_daemon_msg "Stopping system $DAEMON $ARGS daemon"
  /sbin/start-stop-daemon --stop --pidfile $PIDFILE --verbose
  log_end_msg $?
}

# Carry out specific functions when asked to by the system
case "$1" in
  start)
   do_start
    ;;
  stop)
   do_stop
    ;;
  restart|reload|force-reload)
    do_stop
    do_start
    ;;
  status)
    status_of_proc "$DAEMON $ARGS" "$DAEMON $ARGS" && exit 0 || exit $?
    ;;
  *)
    echo "Usage: /etc/init.d/CheckTxPower {start|stop}"
    exit 1
    ;;
esac

exit 0
