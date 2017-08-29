from comptests.registrar import comptest, run_module_tests
from duckietown_utils.system_cmd_imp import system_cmd_result, CmdException
from easy_regression.conditions.interface import RTCheck
import shutil
from duckietown_utils.disk_hierarchy import create_tmpdir


def run(which, expect):
    v = False
    cwd = create_tmpdir('run-regression')
    try:
        cmd = ['rosrun', 'easy_regression', 'run', 
               '--expect', expect, 
               '--test', which,
               '-c', 'rmake']
        system_cmd_result(cwd, cmd,
              display_stdout=v,
              display_stderr=v,
              raise_on_error=True)
    finally:
        shutil.rmtree(cwd)  
    
@comptest
def run_abnormal1():
    run('expect_abnormal1', RTCheck.ABNORMAL)


@comptest
def run_abnormal3():
    run('expect_abnormal3', RTCheck.ABNORMAL)
    
    
@comptest
def run_dontrun1():
    try:
        run('expect_dontrun1', RTCheck.OK)
    except CmdException as e:
        if 'NOT-existing' in e.res.stderr:
            return
        raise
    
    
@comptest
def run_ok1():
    run('expect_ok1', RTCheck.OK)

@comptest
def run_nodata1():
    run('expect_nodata1', RTCheck.NODATA)
@comptest
def run_nodata2():
    run('expect_nodata2', RTCheck.NODATA)

@comptest
def run_fail1():
    run('expect_fail1', RTCheck.FAIL)
    
    
    
if __name__ == '__main__':
    run_module_tests()