#!/usr/bin/env bash
#
# Executed at boot to ensure there is an Access Point setup to
# allow wireless access to the robot. If the AP connection
# doesn't exist, it is created. If it does exist, its SSID
# definition is updated to match the current hardware.
#
# In order to not require hosting a DHCP server on the robot,
# an address is statically assigned to the robot. The requirements
# for the host computer are in
# https://github.com/billmania/roboquest_core/wiki/Robot-WiFi-Access-Point
#

NM_CONN_NAME="roboAP"
AP_SSID_BASE="roboAP"
AP_PSK="roboquest"
AP_IP="192.168.193.1/24"

#
# In order to create a unique Access Point number, append
# the last 4 digits of the eth0 MAC address.
#
UNIQUE_MAC=$(ip address show dev eth0 scope link | \
              awk '/ether/{print $2}' | \
              sed 's/://g' | \
              grep -o '....$')
AP_SSID="${AP_SSID_BASE}_${UNIQUE_MAC}"

if ! nmcli -c no conn show $NM_CONN_NAME > /dev/null 2> /dev/null
then
    #
    # The roboAP does not exist, so create it.
    #
    logger -p user.notice "Defining Access Point:$AP_SSID"
    nmcli -c no \
        conn add \
        type wifi \
        ifname wlan0 \
        con-name $NM_CONN_NAME \
        autoconnect no \
        ssid $AP_SSID
    nmcli -c no \
        conn modify $NM_CONN_NAME \
        802-11-wireless.mode ap \
        802-11-wireless.band bg \
        802-11-wireless.channel 6 \
        802-11-wireless-security.key-mgmt wpa-psk \
        802-11-wireless-security.psk "$AP_PSK" \
        ipv4.addresses "$AP_IP" \
        ipv4.method manual \
        ipv6.method disabled
else
    logger -p user.notice  "Access Point ${NM_CONN_NAME} exists"
    ssid=$(nmcli -c no -t --fields 802-11-wireless.ssid conn show ${NM_CONN_NAME} \
            | cut -d: -f2)
    if [[ "${ssid}" != "${AP_SSID}" ]]
    then
        nmcli -c no conn mod ${NM_CONN_NAME} 802-11-wireless.ssid ${AP_SSID}
        logger -p user.notice  " Set SSID to ${AP_SSID}"
    fi

fi

nmcli -c no \
    conn up ifname wlan0

exit 0
