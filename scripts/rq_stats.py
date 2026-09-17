#!/usr/bin/env python3
"""Collect the ID and software stats.

The Roboquest robots record some statistics about themselves.
Extract the newest information and publish it to an HTML file.
"""

import argparse
import logging
from datetime import datetime
from json import dump, dumps, loads
from os import fdopen as os_fdopen
from os import replace as os_replace
from pathlib import Path
from re import compile
from sys import exit as sys_exit
from tempfile import mkstemp
from time import time

from dateutil.tz import gettz

from platformdirs import user_state_dir

VERSION = '2.1'
LOG_DIR = '/var/log/lighttpd'
RQ_STATS_LOG = '/var/log/rq_stats.log'
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
        logging.basicConfig(
            filename=RQ_STATS_LOG,
            format='%(asctime)s %(levelname)s %(message)s',
            level=logging.DEBUG)
        logging.info(f'rq_stats.py version {VERSION} started')

        self._parsed_args = self._parse_args()
        self._stats = {}

    def _parse_args(self) -> argparse.Namespace:
        """Parse the arguments from the command line."""
        parser = argparse.ArgumentParser(
            prog='rq_stats.py',
            description='Collate the robot details',
            epilog=''
        )

        parser.add_argument(
            '--time_rollback',
            dest='time_rollback',
            default=0,
            help='How many minutes to rollback the last timestamp',
            type=int
        )
        parser.add_argument(
            '--show_stats',
            dest='show_stats',
            action='store_true',
            default=False,
            help='Just display the historical stats'
        )

        return parser.parse_args()

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
        if self._parsed_args.time_rollback:
            self._last_timestamp -= (self._parsed_args.time_rollback * 60.0)
            logging.warning(
                'Last timestamp was rolled back'
                f' {self._parsed_args.time_rollback} minutes.'
            )

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

        entry is expected to contain a query string. The first key must
        be 'serial'.

        Return the key-values as a dictionary.
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
        versions_re = compile('firmware.*ui=')
        register_re = compile('register.*user=')

        robots = self._stats['robots']
        for log_file in self._logs_list:
            for log_entry in log_file.read_text().splitlines():
                entry_timestamp = self._parse_timestamp(log_entry[1:27])
                if entry_timestamp > self._last_timestamp:
                    if versions_re.search(log_entry):
                        #
                        # This log entry has software version strings.
                        #
                        versions = self._parse_entry(log_entry)
                        user = ''
                        if (versions['serial'] in robots
                           and 'user' in robots[versions['serial']]):
                            user = robots[versions['serial']]['user']

                        if (
                            versions['serial'] not in robots
                            or (
                                robots[versions['serial']]['timestamp']
                                < entry_timestamp
                            )
                           ):
                            robots[versions['serial']] = {
                                'timestamp': entry_timestamp,
                                'user': user,
                                'id': versions['id'],
                                'updater': versions['updater'],
                                'core': versions['core'],
                                'ui': versions['ui']
                            }

                        #
                        # An individual log entry can't be both types.
                        #
                        continue

                    if register_re.search(log_entry):
                        #
                        # This log entry has a user registration string.
                        #
                        registration = self._parse_entry(log_entry)
                        if registration['serial'] not in robots:
                            #
                            # This is the first log entry ever seen for
                            # this robot.
                            #
                            logging.debug(
                                '_process_logs: first entry was registration'
                                f" serial: {registration['serial']}"
                            )
                            robots[registration['serial']] = {
                                'timestamp': entry_timestamp,
                                'user': registration['user'],
                                'id': '',
                                'updater': '',
                                'core': '',
                                'ui': ''
                            }
                        elif (
                            robots[registration['serial']]['timestamp']
                                < entry_timestamp
                           ):
                            logging.debug(
                                '_process_logs: New registration'
                                f" serial: {registration['serial']}"
                                f" stats: {robots[registration['serial']]}"
                            )
                            robots[registration['serial']]['user'] = (
                                registration['user']
                            )
                            robots[registration['serial']]['timestamp'] = (
                                entry_timestamp
                            )

                        continue

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
            f.write('<!DOCTYPE html><html><head>')
            f.write('<title>RQ robots status</title>')
            f.write('</head>\n<body>\n')
            isotime = datetime.fromtimestamp(
                self._stats['last_timestamp']
            ).astimezone().isoformat()
            f.write(f'<h1>Status as of {isotime}</h1>')
            f.write('<table border=2><tr>\n')
            f.write('<th>Serial</th>\n')
            f.write('<th>Last seen</th>\n')
            f.write('<th>User</th>\n')
            f.write('<th>ID</th>\n')
            f.write('<th>updater</th>\n')
            f.write('<th>rq_core</th>\n')
            f.write('<th>rq_ui</th>\n')
            f.write('</tr>\n')

            robots = self._stats['robots']
            for serial in robots:
                try:
                    f.write('<tr>')
                    f.write(f'<td>{serial}</td>')
                    isotime = datetime.fromtimestamp(
                        robots[serial]['timestamp']
                    ).astimezone().isoformat()
                    f.write(f'<td>{isotime}')
                    f.write(f"<td>{robots[serial]['user']}</td>")
                    f.write(f"<td>{robots[serial]['id']}</td>")
                    f.write(f"<td>{robots[serial]['updater']}</td>")
                    f.write(f"<td>{robots[serial]['core']}</td>")
                    f.write(f"<td>{robots[serial]['ui']}</td>")
                    f.write('</tr>\n')

                except Exception as e:
                    logging.warning(
                        '_write_html:'
                        f' Exception: {e}'
                    )
                    logging.warning(
                        '_write_html:'
                        f' serial: {serial}'
                        f' stats: {robots[serial]}'
                    )

            f.write('</table></body></html>\n')

        return

    def _show_stats(self):
        """Display the historical stats.

        Pretty print the contents of the self._stats object.
        """
        stats_pp = dumps(
            self._stats,
            indent=2,
            sort_keys=True
        )
        print(f'{stats_pp}')

    def run(self):
        """Run the process."""
        self._load_stats()

        if self._parsed_args.show_stats:
            self._show_stats()
            sys_exit(0)

        self._find_logs()

        self._process_logs()

        self._save_stats()

        self._write_html()

        pass


if __name__ == '__main__':
    rq_stats = RQStats()
    rq_stats.run()
