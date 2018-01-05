import duckietown_utils as dtu
import os
from compmake.utils.filesystem_utils import mkdirs_thread_safe



@dtu.unit_test
def test_thumbnails():
    id_log = '2016-04-29-dp3auto-neptunus-1'
    out = dtu.get_output_dir_for_test()
    cmd = ['/opt/ros/kinetic/bin/rosrun', 'easy_logs', 'thumbnails', '-o', out, id_log, '-c', 'rmake']
    run_one(cmd)
    
@dtu.unit_test
def test_videos():
    id_log = '2016-04-29-dp3auto-neptunus-1/{1:3}'
    out = dtu.get_output_dir_for_test()
    
    cmd = ['rosrun', 'easy_logs', 'videos', '-o', out, id_log, '-c', 'rmake']
    run_one(cmd)
    
def run_one(cmd):
    v = False
    cwd = dtu.get_output_dir_for_test()
    if not os.path.exists(cwd):
        mkdirs_thread_safe(cwd)
    try:
        dtu.system_cmd_result(cwd, cmd,
              display_stdout=v,
              display_stderr=v,
              raise_on_error=True)
    finally:
        pass
#         if os.path.exists(cwd):
#             shutil.rmtree(cwd)  
    