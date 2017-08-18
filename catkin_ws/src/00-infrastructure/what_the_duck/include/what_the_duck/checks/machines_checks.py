import os
import socket

from duckietown_utils.constants import get_machines_files_path,\
    get_scuderia_path
from duckietown_utils.exception_utils import raise_wrapped
from duckietown_utils.exceptions import DTConfigException
from what_the_duck.check import Check, CheckFailed, CheckError


class MachinesExists(Check):

    def check(self):
        try:
            path = get_machines_files_path()
        except DTConfigException as e:
            msg = 'Could not get path to machines.'
            raise_wrapped(CheckError, e, msg)

        if not os.path.exists(path):
            msg = 'Machines file does not exist.'
            l = 'File does not exist: %s ' % path
            raise CheckFailed(msg, l)


class MachinesNewerThanScuderia(Check):

    def check(self):
        try:
            path_machines = get_machines_files_path()
            path_scuderia = get_scuderia_path()
        except DTConfigException as e:
            msg = 'Could not get path to machines.'
            raise_wrapped(CheckError, e, msg)

        if not os.path.exists(path_machines):
            msg = 'Machines file does not exist.'
            raise CheckError(msg)

        if not os.path.exists(path_scuderia):
            msg = 'Scuderia file does not exist.'
            raise CheckError(msg)

        f1 = os.path.getmtime(path_machines)
        f2 = os.path.getmtime(path_scuderia)

        if f2 > f1:
            msg = 'The machines file is older than the scuderia file.'
            raise CheckFailed(msg)


class MachinesValid(Check):

    def check(self):
        try:
            path = get_machines_files_path()
        except DTConfigException as e:
            msg = 'Could not get path to machines.'
            raise_wrapped(CheckError, e, msg)

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
