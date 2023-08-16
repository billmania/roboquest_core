#!/usr/bin/env bash
#
# Bill Mania, 15 Aug 2023
#
# Executed at boot to ensure the hostname is unique,
# before requesting an IP address from the DHCP service.
# It must be executed by systemd, not /etc/rc.local.
#

HOSTNAME_BASE="rq"

logger -p user.notice  "hostname.bash started"
printf "hostname.bash was started\n" >&2

#
# In order to create a unique hostname, append
# the last 4 digits of the eth0 MAC address.
#
UNIQUE_HOST=$(ip address show dev eth0 scope link | \
	       awk '/ether/{print $2}' | \
	       sed 's/://g' | \
	       grep -o '....$')
HOSTNAME="$HOSTNAME_BASE-$UNIQUE_HOST"

hostnamectl set-hostname ${HOSTNAME}

logger -p user.notice  "Hostname set to $HOSTNAME"
printf "Hostname set to %s\n" $HOSTNAME >&2

exit 0
