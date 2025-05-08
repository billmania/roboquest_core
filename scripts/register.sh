#!/usr/bin/env bash

#
# Associate a user name with the robot's serial number. 
#

SERIAL_NUMBER_FILE="/sys/firmware/devicetree/base/serial-number"
REGISTER_URL="https://registry.q4excellence.com:5678/register"

get_serial ()
{
    cat $SERIAL_NUMBER_FILE
}

get_user_name ()
{
    echo "$1" | sed -e 's/[^a-zA-Z0-9]//g' -e 's/^\(.........................\).*$/\1/'
}

register_robot ()
{
        curl ${REGISTER_URL}?serial="$1"\&user="$2"
}

raw_user_name=$1

register_robot $(get_serial) $(get_user_name "${raw_user_name}")

exit 0
