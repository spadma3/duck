import os
import socket
import duckietown_utils as dtu

from what_the_duck.check import Check, CheckFailed, CheckError

__all__ = [
    'MachinesExists',
    'MachinesValid',
]

class MachinesExists(Check):

    def check(self):
        try:
            path = dtu.get_machines_files_path()
        except dtu.DTConfigException as e:
            msg = 'Could not get path to machines.'
            dtu.raise_wrapped(CheckError, e, msg)

        if not os.path.exists(path):
            msg = 'Machines file does not exist.'
            l = 'File does not exist: %s ' % path
            raise CheckFailed(msg, l)


class MachinesValid(Check):

    def check(self):
        try:
            path = dtu.get_machines_files_path()
        except dtu.DTConfigException as e:
            msg = 'Could not get path to machines.'
            dtu.raise_wrapped(CheckError, e, msg)

        if not os.path.exists(path):
            msg = 'Machines file does not exist.'
            l = 'File does not exist: %s ' % path
            raise CheckFailed(msg, l)

        hostname = socket.gethostname()
        contents = open(path).read()

        if not '"%s"' % hostname in contents:
            msg = 'This robot, "%s" is not mentioned in the machines file.' % hostname
            raise CheckFailed(msg)

        # TODO: read the machines
