import duckietown_utils as dtu

from duckietown_utils import system_cmd_result


@dtu.unit_test
def run1():
    cmd = ['rosrun', 'duckieteam', 'create-machines', '--print']
    cwd = '.'
    system_cmd_result(cwd,cmd,
                      display_stdout=False,
                      display_stderr=False,
                      raise_on_error=True)

@dtu.unit_test
def run2():
    tmpfile = '/tmp/tmp'
    cmd = ['rosrun', 'duckieteam', 'create-roster', '--roster', tmpfile]
    cwd = '.'
    system_cmd_result(cwd,cmd,
                      display_stdout=False,
                      display_stderr=False,
                      raise_on_error=True)

@dtu.unit_test
def run():
    cmd = ['rosrun', 'duckieteam', 'create-roster']
    cwd = '.'
    system_cmd_result(cwd,cmd,
                      display_stdout=False,
                      display_stderr=False,
                      raise_on_error=True)

if __name__ == '__main__':
    dtu.run_tests_for_this_module()
