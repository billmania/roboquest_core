# Adding a new WiFi connection

By default, the Raspberry Pi has two network connections defined which
use the WiFi interface. There is a Network Manager Connection using the
Raspberry Pi as a WiFi Access Point and another Connection for a client
on the school's VCS wireless network.

Here, the procedure for defining a new WiFi client Connection is described.

1. Using the HAT UI, find the IP address on screen 2 for a connection.
2. Use your ssh client to connect with the Raspberry Pi.
3. As the superuser (sudo -i) run the nmtui command line program.
    1. Choose Edit a connection
    2. Choose Add
    3. Choose Wi-Fi
    4. for Profile name use the SSID
    5. for SSID use the SSID
    6. for Security, choose according to your WiFi router
    7. for Password, if requested, provide your WiFi password
    8. leave everything else at the default setting
    9. Choose OK
    10. verify your new WiFi connection now appears in the nmcli list
    11. Choose Back, Quit, then OK
4. Go to HAT UI screen 3 and find your SSID
5. Disconnect your ssh session
6. Go to HAT UI screen 2 and disable (press Enter) any WiFi Connection(s)
7. If the new WiFi connection still appears on screen 3, press the Enter key
8. Verify the new WiFi connection appears on HAT UI screen 2

