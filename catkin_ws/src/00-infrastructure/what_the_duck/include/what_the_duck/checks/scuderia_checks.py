from what_the_duck.check import Check, CheckFailed
import socket
from duckieteam.cli.create_machines import get_scuderia_contents

__all__ = ['ThisRobotInScuderiaFile']

class ThisRobotInScuderiaFile(Check):

    def check(self): 
        contents = get_scuderia_contents()
        robot_name = socket.gethostname()
        if not robot_name in contents:
            msg = 'There is no entry "%s" in the scuderia file.' % robot_name
            raise CheckFailed(msg)

