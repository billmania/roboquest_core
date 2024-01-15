#!/usr/bin/env bash

#
# Send an action to the RoboQuest software updater.
#
# Usage is:
#   update_cli.bash {action} {args}
#

VERSION=2

ACTION="${1:-UPDATE}"
shift 1
ARGS=$*

UPDATE_FIFO="/tmp/update_fifo"
COMMAND="{\"timestamp\": $(date +%s), \"version\": ${VERSION}, \"action\": \"${ACTION}\", \"args\": \"${ARGS}\"}"

if [[ -p "$UPDATE_FIFO" && -w "$UPDATE_FIFO" ]]
then
        printf "$COMMAND" > $UPDATE_FIFO
        printf "${ACTION} command sent\n"
        exit 0
fi

printf "$UPDATE_FIFO does not exist as a writable named pipe\n" >&2
printf "updater.py may not be running or you may not be superuser\n" >&2
exit 1
