import duckietown_utils as dtu



@dtu.unit_test
def run1():
    cmd = ['rosrun', 'duckieteam', 'create-machines', '--print']
    cwd = dtu.get_output_dir_for_test()
    dtu.system_cmd_result(cwd,cmd,
                      display_stdout=False,
                      display_stderr=False,
                      raise_on_error=True)

@dtu.unit_test
def run2():
    tmpfile = 'roster.html'
    cmd = ['rosrun', 'duckieteam', 'create-roster', '--roster', tmpfile]
    cwd = dtu.get_output_dir_for_test()
    dtu.system_cmd_result(cwd,cmd,
                      display_stdout=False,
                      display_stderr=False,
                      raise_on_error=True)

@dtu.unit_test
def run():
    cmd = ['rosrun', 'duckieteam', 'create-roster']
    cwd = dtu.get_output_dir_for_test()
    dtu.system_cmd_result(cwd,cmd,
                      display_stdout=False,
                      display_stderr=False,
                      raise_on_error=True)

if __name__ == '__main__':
    dtu.run_tests_for_this_module()
