#!/usr/bin/env bash

#
# Associate a user name with the robot's serial number. 
#

SERIAL_NUMBER_FILE="/sys/firmware/devicetree/base/serial-number"
REGISTER_URL="https://registry.q4excellence.com:5678/register"

get_serial ()
{
    sed -e 's/\x00$//' < $SERIAL_NUMBER_FILE
}

get_user_name ()
{
    echo "$1" | sed -e 's/[^a-zA-Z0-9]//g' -e 's/^\(.........................\).*$/\1/'
}

register_robot ()
{
        curl ${REGISTER_URL}?serial="$1"\&user="$2"
}

user_name=$(get_user_name "$1")
serial_number=$(get_serial)
register_robot "${serial_number}" "${user_name}"

echo "Registered user ${user_name}"

exit 0
