from typing import Callable
import subprocess

from roboquest_core.rq_hat import HAT_SCREEN, HAT_BUTTON, EOL
from roboquest_core.rq_hat import SCREEN_HEADER

SUBPROCESS_TIMEOUT_S = 3.0
FIELD_SEP = ':'

NMCLI = '/usr/bin/nmcli'


class RQNetwork(object):
    """
    Retrieve network information required for the HAT UI screens and
    format it for display on the screens. Also provides the means for
    enabling one of the defined Connections. This class doesn't
    communicate with the HAT, instead relying upon the RQManage class
    as the intermediary.

    Instead of using the python3-networkmanager module, the class
    uses the nmcli utility from the network-manager package. The
    Python module is no longer supported. Using nmcli directly
    makes it easy to test and troubleshoot from the command line,
    in the same way this class uses nmcli.
    """

    def __init__(self,
                 logger: Callable[[str], None],
                 pad_line: Callable[[str], str],
                 pad_text: Callable[[str], str]):
        """
        Initialize the collection of defined Network Manager Connections.

        logger is a callback for handling log entries. Typically it's one
        of the ROS node logging methods.

        pad_line is a callback to pad individual lines of output
        to work with the HAT.

        pad_text fills out the whole block of text to comply with
        the HAT's requirements.
        """

        self._logger = logger
        self._pad_line = pad_line
        self._pad_text = pad_text

        self._screen_page = 0
        self._previous_screen = 0
        self._inactive_connections = list()
        self._inactive_count = 0
        self._active_connections = list()
        self._active_count = 0
        self._connection_to_activate = None
        self._connection_to_deactivate = None

        self._get_all_connections()

    def process_screen_request(self,
                               screen_id: HAT_SCREEN,
                               button_id: HAT_BUTTON) -> str:
        """
        Called when RQManage has determined either that the screen
        has changed or the page of the current screen has changed.
        """

        if screen_id != self._previous_screen:
            self._previous_screen = screen_id
            self._screen_page = 0
            #
            # Because it's possible to change the state of the
            # network Connections outside of this application,
            # refresh the list when the HAT UI screen is changed.
            #
            self._get_all_connections()
        else:
            if button_id != HAT_BUTTON.ENTER:
                self._screen_page += button_id.value

        # TODO: Does the HAT really require the header to be lower case?
        page_info = SCREEN_HEADER.lower()
        page_info += '='
        page_info += screen_id.value
        page_info += '='

        if screen_id == HAT_SCREEN.DEVICES:
            page_info += self._show_active_connection(self._screen_page)
            if button_id != HAT_BUTTON.ENTER:
                page_info += self._show_active_connection(self._screen_page)
            else:
                self._deactivate_connection(self._connection_to_deactivate)

        elif screen_id == HAT_SCREEN.CONNECTIONS:
            if button_id != HAT_BUTTON.ENTER:
                page_info += self._show_inactive_connection(self._screen_page)
            else:
                self._activate_connection(self._connection_to_activate)
        else:
            #
            # This method was called for a SCREEN which doesn't involve
            # network connections.
            #
            page_info += ''

        return page_info

    def _get_all_connections(self) -> None:
        """
        Retrieve the list of all defined Connections, separating them
        into two lists: active and inactive.
        """

        self._inactive_connections = list()
        self._active_connections = list()

        #
        # returns
        # b'roboAP:17740ed8-bf37-4a03-b178-d72652d0e20c:802-11-wireless\
        #  :yes:wlan0\nWired connection 1:d27158e3-5b00-3cd2-9021-aef1d\
        #  a5134c6:802-3-ethernet:yes:eth0\ndocker0:9792665e-73b3-4ca3-\
        #  8d8b-70039a4e58a5:bridge:yes:docker0\nVCS:a06b74f5-f6a2-49bf\
        #  -81a1-9dd555a00b73:802-11-wireless:no:\n'
        #
        output = subprocess.run(
            [NMCLI, '-c', 'no', '-t',
             '-f', 'NAME,UUID,TYPE,ACTIVE,DEVICE',
             'conn'],
            check=True,
            timeout=1.0,
            capture_output=True)
        if output.returncode == 0:
            connections_raw = output.stdout.decode('ascii')
            connections = connections_raw.split(EOL)
            for connection in connections:
                if not connection:
                    continue

                fields = connection.split(FIELD_SEP)
                if len(fields) > 1 and fields[2] == 'bridge':
                    #
                    # bridge connections aren't of any use here
                    #
                    continue

                entry = dict()
                entry['NAME'] = fields[0]
                entry['UUID'] = fields[1]
                entry['TYPE'] = fields[2]
                entry['ACTIVE'] = True if fields[3] == 'yes' else False
                entry['DEVICE'] = fields[4]

                #
                # This subprocess.run must execute as root in order to
                # retrieve the PSK secret.
                #
                # returns
                # b'802-11-wireless.ssid:roboAP_23eb\n802-11-wireless-secur\
                #  ity.psk:roboquest\nGENERAL.NAME:roboAP\nIP4.ADDRESS[1]:1\
                #  0.42.0.1/24\n'
                #
                output = subprocess.run(
                    [NMCLI, '-c', 'no', '-s', '-t',
                     '-f',
                     'GENERAL.NAME,IP4.ADDRESS,802-11-wireless.ssid'
                     + ',802-11-wireless-security.psk,802-11-wireless.mode',
                     'conn',
                     'show', entry['UUID']],
                    check=True,
                    timeout=1.0,
                    capture_output=True)
                if output.returncode == 0:
                    key_values = output.stdout.decode('ascii').split(EOL)
                    for key_value in key_values:
                        if key_value:
                            fields = key_value.split(FIELD_SEP)
                            entry[fields[0]] = fields[1]

                    if entry['ACTIVE']:
                        self._active_connections.append(entry)
                    else:
                        self._inactive_connections.append(entry)

        self._logger(f"ACTIVE: {self._active_connections}")
        self._logger(f"INACTIVE: {self._inactive_connections}")
        self._active_count = len(self._active_connections)
        self._inactive_count = len(self._inactive_connections)

    def _show_active_connection(self, page: int) -> str:

        """
        Retrieve the page-th active connection from the list of Connections
        and return its details, formatted. The details are intended to
        be used by a client to connect to the robot. They're not intended for
        activation or deactivation of a Connection.
        """

        if self._active_count == 0:
            return self._pad_line('No active connections')

        index = page % self._active_count
        self._connection_to_deactivate = self._active_connections[index]

        output = self._format_details(
            self._connection_to_deactivate,
            index,
            'active')

        self._logger(f"show_active-{page}> {output}")
        return output

    def _show_inactive_connection(self, page: int) -> str:
        """
        Retrieve the details for the page-th connection from the list
        of Connections and return them, formatted. The details are used
        to decide which Connection to activate.
        """

        if self._inactive_count == 0:
            return self._pad_line('No inactive connections')

        index = page % self._inactive_count
        self._connection_to_activate = self._inactive_connections[index]

        output = self._format_details(
            self._connection_to_activate,
            index,
            'inactive')

        self._logger(f"show_inactive-{page}> {output}")
        return output

    def _format_details(
            self,
            connection: dict,
            index: int,
            connection_type: str) -> str:
        """
        Accept a connection as input. Extract the details and format
        them suitable for display on the HAT UI. If include_IP
        is True, include the IP address in the output.

        Format the device name, SSID and PSK (if exists), mode,
        and strength.
        """
        # TODO: Clarify what strength is supposed to be.

        output = self._pad_line(f"Name: {connection['DEVICE']}")
        if connection_type == 'active':
            try:
                output += self._pad_line(f"IP: {connection['IP4.ADDRESS[1]']}")
            except Exception as e:
                self._logger(f"IP Exception {e}")

            connections_count = self._active_count
        else:
            connections_count = self._inactive_count

        if '802-11-wireless.ssid' in connection:
            output += self._pad_line(
                f"SSID: {connection['802-11-wireless.ssid']}")
        if '802-11-wireless-security.psk' in connection:
            if connection['802-11-wireless-security.psk']:
                output += self._pad_line(
                    f"Pwd: {connection['802-11-wireless-security.psk']}")
        if '802-11-wireless.mode' in connection:
            output += self._pad_line(
                f"Type: {connection['802-11-wireless.mode']}")

        output = self._pad_line(f"{index + 1}/{connections_count}") + output

        return self._pad_text(output)

    def _deactivate_connection(self, connection: dict) -> bool:
        """
        Deactivate connection so it can no longer be used by a
        client to connect to the robot.
        """

        if not connection:
            return False

        return self._change_connection_state(connection, 'down')

    def _activate_connection(self, connection: dict) -> bool:
        """
        Activate connection so it can be used by a client
        to connect to the robot.
        """

        if not connection:
            return False

        return self._change_connection_state(connection, 'up')

    def _change_connection_state(self,
                                 connection: dict,
                                 new_state: str) -> bool:
        """
        Change the state of connection to new_state and then
        refresh the collections of connections.

        new_state must be in ['up', 'down']
        """

        # TODO: Make the following subprocess.run() asynchronous
        try:
            output = subprocess.run(
                [NMCLI, '-c', 'no', 'conn', new_state,
                 connection['UUID']],
                check=True,
                timeout=SUBPROCESS_TIMEOUT_S,
                capture_output=True)

        except subprocess.TimeoutExpired:
            self._logger(
                f"Timeout changing state of connection {connection['UUID']}")
            return False

        if output.returncode != 0:
            self._logger(
                f"Return code {output.returncode}"
                f" changing state of connection {connection['UUID']}")
            return False

        self._get_all_connections()

        return True
