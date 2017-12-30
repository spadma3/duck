from duckietown_utils.system_cmd_imp import system_cmd_result

import duckietown_utils as dtu

@dtu.unit_test
def test_cli1():
    cwd = '.'
    cmd = ['rosrun', 'easy_algo', 'summary']
    system_cmd_result(cwd, cmd,
                      display_stdout=True,
                      display_stderr=True,
                      raise_on_error=True)
@dtu.unit_test
def test_cli2():
    cwd = '.'
    cmd = ['rosrun', 'easy_algo', 'summary', 'robot']
    system_cmd_result(cwd, cmd,
                      display_stdout=True,
                      display_stderr=True,
                      raise_on_error=True)
