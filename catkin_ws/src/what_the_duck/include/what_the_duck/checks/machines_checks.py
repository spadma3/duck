import os
import os

from duckietown_utils.constants import get_machines_files_path
from duckietown_utils.exception_utils import raise_wrapped
from duckietown_utils.exceptions import DTConfigException
from what_the_duck.check import Check, CheckFailed, CheckError


class ValidMachinesFile(Check):

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

        # TODO: read the machines
