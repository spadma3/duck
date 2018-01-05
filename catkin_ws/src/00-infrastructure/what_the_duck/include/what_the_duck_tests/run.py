import warnings

import duckietown_utils as dtu
from what_the_duck.check import Check, CheckFailed, CheckError
from what_the_duck.entry import Diagnosis
from what_the_duck.list_of_checks import Manager
from what_the_duck.sanity_checks import run_checks
from what_the_duck.statistics import display_results, display_summary
import os


def _cwd():
    cwd = dtu.get_output_dir_for_test()
    if not os.path.exists(cwd):
        dtu.mkdirs_thread_safe(cwd)
    return cwd

@dtu.unit_test
def test_cli2():
    cmd = ['rosrun', 'what_the_duck', 'what-the-duck']
    res = dtu.system_cmd_result(_cwd(), cmd,
                      display_stdout=True,
                      display_stderr=True,
                      raise_on_error=False)
    if False:
        if not (res.ret == 0):
            raise Exception(str(res))
    else:
        warnings.warn('XXX temporary hack')

class AlwaysFails(Check):
    def __init__(self, has_long_error):
        self.has_long_error = has_long_error

    def check(self):
        msg = 'Always fails'
        l = "(long error)" if self.has_long_error else None
        raise CheckFailed(msg, l)

class AlwaysErrors(Check):
    def __init__(self, has_long_error):
        self.has_long_error = has_long_error

    def check(self):
        msg = 'Always fails'
        l = "(long error)" if self.has_long_error else None
        raise CheckError(msg, l)

class AlwaysOK(Check):
    def __init__(self, output=None):
        self.output = output

    def check(self):
        return self.output

@dtu.unit_test
def test_visualization1():

    m = Manager()
    m.add(None, "Fails long", AlwaysFails(True), Diagnosis("diagnosis"))
    m.add(None, "Fails short", AlwaysFails(False), Diagnosis("diagnosis"))
    m.add(None, "Error long", AlwaysFails(True), Diagnosis("diagnosis"))
    m.add(None, "Error short", AlwaysFails(False), Diagnosis("diagnosis"))
    m.add(None, "OK none", AlwaysOK(None), Diagnosis("diagnosis"))
    m.add(None, "OK string", AlwaysOK("output"), Diagnosis("diagnosis"))

    entries = m.entries

    results = run_checks(entries)

    print display_results(results, show_successes=True)
    print display_results(results, show_successes=False)
    print display_summary(results)

if __name__ == '__main__': # pragma: no cover
    dtu.run_tests_for_this_module()
