# RoboQuest software update mechanism

A system for updating the RoboQuest software on the robot, to be
initiated by an event on the UI.

## Requirements

### for the updater

1. electrical power and network connectivity are stable
1. when not updating ensures the containers are running and
   the named pipe is available
1. periodically polls the named pipe for update commands from the UI
   and checks for UPDATING flag
1. updating
    1. use the Screen 4 display for status - not yet implemented
    1. installs latest version of updater and exits
    1. stops the containers
    1. performs a prune of containers
    1. ensure volumes, devices, and filesystem maps - not yet implemented
    1. compares current image version to registry version
    1. if new version exists
        1. pulls latest images from registry
        1. return to monitoring

### for the UI

1. when the UPDATE event occurs
    1. hide all UI widgets
    1. open the named pipe for w
    1. write the UPDATE command
    1. restore and restart UI
