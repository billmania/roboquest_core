#!/usr/bin/env python3
"""Collect the ID and software stats.

The Roboquest robots record some statistics about themselves.
Extract the newest information and publish it to an HTML file.
"""

from datetime import datetime
from json import dump, loads
from os import fdopen as os_fdopen
from os import replace as os_replace
from pathlib import Path
from re import compile
from tempfile import mkstemp
from time import time

from dateutil.tz import gettz

from platformdirs import user_state_dir

VERSION = '1'
LOG_DIR = '/var/log/lighttpd'
STATS_FILE = Path(user_state_dir('rq_stats')) / 'rq_stats.json'
HTML_FILE = '/var/www/html/rq_stats.html'

MONTHS = {
    'Jan': '01',
    'Feb': '02',
    'Mar': '03',
    'Apr': '04',
    'May': '05',
    'Jun': '06',
    'Jul': '07',
    'Aug': '08',
    'Sep': '09',
    'Oct': '10',
    'Nov': '11',
    'Dec': '12'
}


class RQStats(object):
    """Manage the stats."""

    def __init__(self):
        """Do the needful."""
        self._stats = {}

    def _load_stats(self) -> dict:
        """Load the historical stats from the last run."""
        try:
            self._stats = loads(STATS_FILE.read_text())

        except FileNotFoundError:
            STATS_FILE.parent.mkdir(parents=True, exist_ok=True)
            self._stats = {
                'robots': {},
                'last_timestamp': 0
            }

        return

    def _find_logs(self) -> list:
        """Return a list of new log files.

        Use glob-ing to find the access.log files which aren't compressed.
        Then keep those which have been modified since the last run.
        """
        self._last_timestamp = self._stats['last_timestamp']
        self._stats['last_timestamp'] = time()
        self._logs_list = []

        for log_files in [
            Path(LOG_DIR).glob('access.log'),
            Path(LOG_DIR).glob('access.log*[0-9]')
        ]:
            for log_file in log_files:
                if log_file.stat().st_mtime >= self._last_timestamp:
                    self._logs_list.append(log_file)

        return

    def _parse_timestamp(self, timestamp: str) -> int:
        """Parse a string timestamp to the Unix epoch.

        The format of timestamp is expected to be:
            DD/MMM/YYYY:HH:MM:SS
        """
        try:
            day = int(timestamp[:2])
            month = int(MONTHS[timestamp[3:6]])
            year = int(timestamp[7:11])
            hour = int(timestamp[12:14])
            minute = int(timestamp[15:17])
            second = int(timestamp[18:20])

            return datetime(
                year,
                month,
                day,
                hour,
                minute,
                second,
                tzinfo=gettz('America/Los_Angeles')
            ).timestamp()

        except Exception as e:
            print(
                f'Failed to parse {timestamp}'
                f', Exception {e}'
            )
            return None

    def _parse_entry(self, entry: str) -> dict:
        """Parse the log entry.

        Extract:
            serial
            id
            updater
            core
            ui

        and return those details as a dictionary.

        An entry looks like:
            [15/Aug/2026:07:32:26 -0700] 75.100.206.94 GET
            /firmware_version.txt?serial=10000000179a7777&id=72&uptime=32.57&updater=21&core=25&ui=36
            HTTP/1.1 200 "python-requests/2.32.3"
        """
        details_section = entry[
            entry.find('serial='): entry.find(' HTTP')
        ]
        details = {}
        for detail_pair in details_section.split('&'):
            detail_key_value = detail_pair.split('=')
            details[detail_key_value[0]] = detail_key_value[1]

        return details

    def _process_logs(self) -> None:
        """Read the entries from the log files.

        Update self._stats with details from newer log
        entries.
        """
        pattern = compile('firmware.*ui=')

        robots = self._stats['robots']
        for log_file in self._logs_list:
            for log_entry in log_file.read_text().splitlines():
                entry_timestamp = self._parse_timestamp(log_entry[1:27])
                if entry_timestamp > self._last_timestamp:
                    if pattern.search(log_entry):
                        robot = self._parse_entry(log_entry)
                        if (
                            robot['serial'] not in robots
                            or (
                                robots[robot['serial']]['timestamp']
                                < entry_timestamp
                            )
                           ):
                            robots[robot['serial']] = {
                                'timestamp': entry_timestamp,
                                'id': robot['id'],
                                'updater': robot['updater'],
                                'core': robot['core'],
                                'ui': robot['ui']
                            }

    def _save_stats(self) -> None:
        """Write the updated stats to the file."""
        fd, tmp = mkstemp(dir=STATS_FILE.parent)
        with os_fdopen(fd, 'w') as f:
            dump(self._stats, f)
        os_replace(tmp, STATS_FILE)

        return

    def _write_html(self) -> None:
        """Write the updated stats to the HTML file."""
        with open(HTML_FILE, 'w') as f:
            f.write('<!DOCTYPE html><html><body>\n')
            f.write('<table border=2><tr>\n')
            f.write('<th>Serial</th>\n')
            f.write('<th>Last seen</th>\n')
            f.write('<th>ID</th>\n')
            f.write('<th>updater</th>\n')
            f.write('<th>rq_core</th>\n')
            f.write('<th>rq_ui</th>\n')
            f.write('</tr>\n')

            robots = self._stats['robots']
            for serial in robots:
                f.write('<tr>')
                f.write(f'<td>{serial}</td>')
                f.write(f"<td>{robots[serial]['timestamp']}</td>")
                f.write(f"<td>{robots[serial]['id']}</td>")
                f.write(f"<td>{robots[serial]['updater']}</td>")
                f.write(f"<td>{robots[serial]['core']}</td>")
                f.write(f"<td>{robots[serial]['ui']}</td>")
                f.write('</tr>\n')

            f.write('</table></body></html>\n')

        return

    def run(self):
        """Run the process."""
        self._load_stats()

        self._find_logs()

        self._process_logs()

        self._save_stats()

        self._write_html()

        pass


if __name__ == '__main__':
    rq_stats = RQStats()
    rq_stats.run()
