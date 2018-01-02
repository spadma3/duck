import duckietown_utils as dtu

@dtu.unit_test
def test_cli1():
    cwd = '.'
    cmd = ['rosrun', 'easy_algo', 'summary']
    dtu.system_cmd_result(cwd, cmd,
                      display_stdout=True,
                      display_stderr=True,
                      raise_on_error=True)
@dtu.unit_test
def test_cli2():
    cwd = '.'
    cmd = ['rosrun', 'easy_algo', 'summary', 'robot']
    dtu.system_cmd_result(cwd, cmd,
                      display_stdout=True,
                      display_stderr=True,
                      raise_on_error=True)
