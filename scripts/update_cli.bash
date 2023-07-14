#!/usr/bin/env bash

#
# Send the UPDATE command to the RoboQuest software updater.
#

VERSION=1
UPDATE_FIFO="/tmp/update_fifo"
COMMAND="{\"timestamp\": $(date +%s), \"version\": ${VERSION}, \"action\": \"UPDATE\", \"args\": \"\"}"

if [[ -p "$UPDATE_FIFO" && -w "$UPDATE_FIFO" ]]
then
        printf "$COMMAND" > $UPDATE_FIFO
        printf "UPDATE command sent\n"
        exit 0
fi

printf "$UPDATE_FIFO does not exist as a writable named pipe\n" >&2
printf "updater.py may not be running or you may not be superuser\n" >&2
exit 1
