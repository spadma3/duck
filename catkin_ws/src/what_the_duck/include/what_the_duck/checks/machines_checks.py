import os
import os

from duckietown_utils.constants import get_machines_files_path
from duckietown_utils.exception_utils import raise_wrapped
from duckietown_utils.exceptions import DTConfigException
from what_the_duck.check import Check, CheckFailed, CheckError
import socket


class MachinesExists(Check):

    def check(self):
        try:
            path = get_machines_files_path()
        except DTConfigException as e:
            msg = 'Could not get path to machines.'
            raise_wrapped(CheckError, e, msg)

        if not os.path.exists(path):
            msg = 'Machines file does not exist.'
            l = 'File does not exist: %s '% path
            raise CheckFailed(msg, l)


class MachinesValid(Check):

    def check(self):
        try:
            path = get_machines_files_path()
        except DTConfigException as e:
            msg = 'Could not get path to machines.'
            raise_wrapped(CheckError, e, msg)
        
        if not os.path.exists(path):
            msg = 'Machines file does not exist.'
            l = 'File does not exist: %s '% path
            raise CheckFailed(msg, l)

        hostname = socket.gethostname()
        contents = open(path).read()
        
        if not hostname in contents: 
            msg = 'This robot, "%s" is not mentioned in the machines file.'
            raise CheckFailed(msg)
        
        # TODO: read the machines
