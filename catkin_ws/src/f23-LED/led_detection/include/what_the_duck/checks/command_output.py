from duckietown_utils.system_cmd_imp import system_cmd_result, CmdException,\
    indent
from what_the_duck.check import Check, CheckError, CheckFailed


__all__ = ['CommandOutputContains']

class CommandOutputContains(Check):
    """ Checks that the command given returns 0
        and the output contains a certain string. """
    def __init__(self, cmd, substring, cwd=None):
        self.cmd = cmd
        self.cwd = cwd
        self.substring = substring
    
    def check(self):
        try:
            res = system_cmd_result(self.cwd, self.cmd,
                      display_stdout=False,
                      display_stderr=False,
                      raise_on_error=True,
                      capture_keyboard_interrupt=False,
                      env=None)
        except CmdException as e:
            msg = 'The command failed\n'
            msg += '\n'+ indent(e, ' >')
            raise CheckError(msg)
    
        if not self.substring in res.stdout:
            compact = ('Could not find string "%s" in output of %s' % 
                       (self.substring, self.cmd))
            long_explanation = 'Complete output is:\n\n' + indent(res.stdout,' > ')
            raise CheckFailed(compact, long_explanation)
