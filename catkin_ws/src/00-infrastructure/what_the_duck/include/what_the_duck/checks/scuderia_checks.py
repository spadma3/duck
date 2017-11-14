# import os
# 
# from duckietown_utils.constants import \
#     get_scuderia_path
# from duckietown_utils.exception_utils import raise_wrapped
# from duckietown_utils.exceptions import DTConfigException
from what_the_duck.check import Check, CheckFailed
import socket
from duckieteam.cli.create_machines import get_scuderia_contents
# 
# class ScuderiaFileExists(Check):
#     
#     def check(self):
#         try:
#             path = get_scuderia_path()
#         except DTConfigException as e:
#             msg = 'Could not get path to machines.'
#             raise_wrapped(CheckError, e, msg)
# 
#         if not os.path.exists(path):
#             msg = 'Scuderia file does not exist.'
#             l = 'File does not exist: %s '% path
#             raise CheckFailed(msg, l)
#         
#  
# class ValidScuderiaFile(Check):
# 
#     def check(self): 
#         try:
#             contents = get_scuderia_contents()
#         except ScuderiaException as e:
#             msg  = 'Invalid scuderia file.'
#             l = str(e)
#             raise CheckFailed(msg, l)
# 
#         if not contents:
#             msg = 'Empty scuderia file.'
#             l = 'The file %s is empty.'
#             raise CheckFailed(msg, l)


class ThisRobotInScuderiaFile(Check):

    def check(self): 
        contents = get_scuderia_contents()
        robot_name = socket.gethostname()
        if not robot_name in contents:
            msg = 'There is no entry "%s" in the scuderia file.' % robot_name
            raise CheckFailed(msg)

