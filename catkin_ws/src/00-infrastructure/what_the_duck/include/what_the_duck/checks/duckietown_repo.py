from what_the_duck.check import Check, CheckFailed


class CheckImportMessages(Check):
    def __init__(self):
        pass    
    
    def check(self):
        try:
            from duckietown_msgs.msg import (AntiInstagramTransform, BoolStamped, Segment, # @UnresolvedImport @UnusedImport
             SegmentList, Vector2D)  # @UnresolvedImport @UnusedImport
        except ImportError as e:
            msg = str(e)
            msg += '\n\n This can usually be fixed by building everything ("make build").'
            msg += '\nOr in extreme cases, "make catkin-clean build".'
            raise CheckFailed(msg)
    
    
    
# 
#         
# class CheckEnvironmentVariables(Check):
#     def __init__(self):
#         pass
# 
#     def check(self):
#     
#         vs = {
#         'DUCKIETOWN_ROOT': """
#     DUCKIETOWN_ROOT should be set.
#         """,
#               'DUCKIETOWN_DATA': """
#     The environment variable DUCKIETOWN_DATA must either:
#     1) be set to "n/a"
#     2) point to an existing path corresponding to Dropbox/duckietown-data.
#         (containing a subdirectory 'logs')
#                     """, 
#             'VEHICLE_NAME':
#                 "The environment variable VEHICLE_NAME must be the name of your robot \n"
#                 " (if you are on the robot). Please add this line to ~/.bashrc: \n"
#                 " \n"
#                 "  export VEHICLE_NAME=<your vehicle name>\n"
#         }
#     
#         # Only check this if we are on the robot
#     
#         if not on_duckiebot():
#             del vs['VEHICLE_NAME']
#     
#         # do not check DUCKIETOWN_DATA on robot
#         if on_duckiebot():
#             del vs['DUCKIETOWN_DATA']
#     
#         errors = []
#         for v in vs:
#             if not v in os.environ:
#                 e = 'Environment variable %r not defined.' % v
#                 errors.append(e + '\n' + vs[v])
#             
#         if not on_duckiebot():
#             if 'DUCKIETOWN_DATA' in os.environ:
#                 path = os.environ['DUCKIETOWN_DATA']
#                 if path != 'n/a':
#                     f = expand_environment(path)
#                     logs = os.path.join(f, 'logs')
#                     if not os.path.exists(f) or not os.path.exists(logs):
#                         e = vs['DUCKIETOWN_DATA']
#                         errors.append(e)
#     
#         if errors:
#             raise CheckFailed('\n---\n'.join(errors))

