import duckietown_utils as dtu
import os



def _cwd():
    cwd = dtu.get_output_dir_for_test()
    if not os.path.exists(cwd):
        dtu.mkdirs_thread_safe(cwd)
    return cwd

@dtu.unit_test
def run1():
    cmd = ['rosrun', 'duckieteam', 'create-machines', '--print']
    dtu.system_cmd_result(_cwd(),cmd,
                      display_stdout=False,
                      display_stderr=False,
                      raise_on_error=True)

@dtu.unit_test
def run2():
    tmpfile = 'roster.html'
    cmd = ['rosrun', 'duckieteam', 'create-roster', '--roster', tmpfile]
    dtu.system_cmd_result(_cwd(),cmd,
                      display_stdout=False,
                      display_stderr=False,
                      raise_on_error=True)

@dtu.unit_test
def run():
    cmd = ['rosrun', 'duckieteam', 'create-roster']
    cwd = dtu.get_output_dir_for_test()
    dtu.system_cmd_result(_cwd(),cmd,
                      display_stdout=False,
                      display_stderr=False,
                      raise_on_error=True)

if __name__ == '__main__':
    dtu.run_tests_for_this_module()
