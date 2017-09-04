from comptests.registrar import comptest
from duckietown_utils.system_cmd_imp import system_cmd_result

@comptest
def test_cli2():
    cwd = '.'
    cmd = ['rosrun', 'what_the_duck', 'what-the-duck']
    res = system_cmd_result(cwd, cmd,
                      display_stdout=True,
                      display_stderr=True,
                      raise_on_error=False)
    assert res.ret > 0
