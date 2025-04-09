#!/usr/bin/env bash

#
# Retrieve an archive of the Roboquest configuration files
# using a GET from the back location. Restore the files
# to the robot's filesystem.
#

TMP_DIR="/tmp"
PERSIST_DIR="/opt/persist"
RESTORE_URL="https://registry.q4excellence.com:5678/archives"
SERIAL_NUMBER_FILE="/sys/firmware/devicetree/base/serial-number"

get_serial ()
{
    cat $SERIAL_NUMBER_FILE
}

name_tar_file ()
{
    TAR_FILE=$(get_serial)_config.tgz
    echo "${TAR_FILE}"
}

retrieve_files ()
{
    curl -X GET --output "$1" "${RESTORE_URL}/$1" 
}

extract_files ()
{
    cd $PERSIST_DIR || return
    tar xzf "${TMP_DIR}/$1"
}

tar_file=$(name_tar_file)
retrieve_files "$tar_file"
extract_files "$tar_file"

exit 0
