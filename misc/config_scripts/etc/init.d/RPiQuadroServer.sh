#!/bin/bash

### BEGIN INIT INFO
# Provides:          RPiQuadroServer
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Put a short description of the service here
# Description:       Put a long description of the service here
### END INIT INFO

WORK_DIR="/var/lib/RPiQuadroServer"
DAEMON="/usr/bin/python"
ARGS="/usr/local/bin/RPiQuadroServer.py"
PIDFILE="/var/run/RPiQuadroServer.pid"
USER="root"

. /lib/lsb/init-functions

do_start() {
  log_daemon_msg "Starting system $DAEMON $ARGS daemon"
  mkdir -p "$WORK_DIR"
  /sbin/start-stop-daemon --start --pidfile $PIDFILE \
    --user $USER --group $USER \
    -b --make-pidfile \
    --chuid $USER \
    --nicelevel -20 \
    --exec $DAEMON $ARGS
  log_end_msg $?
}

do_stop() {
  log_daemon_msg "Stopping system $DAEMON $ARGS daemon"
  /sbin/start-stop-daemon --stop --pidfile $PIDFILE --verbose
  log_end_msg $?
}

case "$1" in
  start)
    do_start
    ;;
  stop)
    log_daemon_msg "Stopping system $DAEMON $ARGS daemon"
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
    echo "Usage: /etc/init.d/$USER {start|stop|restart|status}"
    exit 1
    ;;
esac