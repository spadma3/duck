import duckietown_utils as dtu

@dtu.unit_test
def test_cli1():
    cwd = dtu.get_output_dir_for_test()
    cmd = ['rosrun', 'easy_algo', 'summary']
    dtu.system_cmd_result(cwd, cmd,
                      display_stdout=True,
                      display_stderr=True,
                      raise_on_error=True)
@dtu.unit_test
def test_cli2():
    cwd = dtu.get_output_dir_for_test()
    cmd = ['rosrun', 'easy_algo', 'summary', 'robot']
    dtu.system_cmd_result(cwd, cmd,
                      display_stdout=True,
                      display_stderr=True,
                      raise_on_error=True)
