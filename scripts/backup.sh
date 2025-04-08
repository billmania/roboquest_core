#!/usr/bin/env bash

#
# Create an archive of the Roboquest configuration files and then
# POST them to the backup location of the backup server.
#

TMP_DIR="/tmp"
PERSIST_DIR="/opt/persist"
BACKUP_URL="https://registry.q4excellence.com:5678/backup"
SERIAL_NUMBER_FILE="/sys/firmware/devicetree/base/serial-number"
FILES_TO_BACKUP="ui/ros_interfaces.js ui/widget_interface.js nodes/user_nodes.launch.py configuration.json servos_config.json i2c/i2c.yaml"

get_serial ()
{
    cat $SERIAL_NUMBER_FILE
}

name_tar_file ()
{
    local serial
    serial=$(get_serial)

    TAR_FILE=${serial}_config

    echo "${TAR_FILE}"
}

collect_files ()
{
    cd $PERSIST_DIR || return
    tar czf "${TMP_DIR}/$1.tgz" ${FILES_TO_BACKUP}
}

save_files ()
{
    curl -l -H 'Content-type: application/gzip' --data-binary "@${TMP_DIR}/$1.tgz" ${BACKUP_URL}?$1
}

tar_file=$(name_tar_file)
collect_files "$tar_file"
save_files "$tar_file"

exit 0

