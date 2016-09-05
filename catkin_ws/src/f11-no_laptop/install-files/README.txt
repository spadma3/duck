Files in this directory are copied to /etc/ upon installation of this module. 

supervisor:

    SystemV init script run run on startup. In ensures supervisord
    starts up when ubuntu does.

supervisord.conf:
    configuration file for supervisord. This file points to the
    f11-no_laptop directory containing this folder. More specifically,
    it tells supervisor to start ../joystick.daemon.py as user=ubuntu
    (rather than, say, user=root.)
