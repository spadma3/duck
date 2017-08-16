from duckietown_utils.system_cmd_imp import system_cmd_result
from what_the_duck.check import Check, CheckFailed
from what_the_duck.checks.fileutils import raise_CheckError_from_CommandResult,\
    summary_of_cmdres


class GithubLogin(Check):
    
    def check(self):
        
        cmd = ['ssh', '-T', 'git@github.com']
        
        res = system_cmd_result(None, cmd,
                  display_stdout=False,
                  display_stderr=False,
                  raise_on_error=False,
                  capture_keyboard_interrupt=True, # XXX?
                  env=None)
        
        expect = 'successfully authenticated'
        if res.ret == 1 and expect in res.stderr:
            # ok
            return
        
        if res.ret == 255 and 'denied' in res.stderr:
            msg = 'You do not have access to Github. '
            l = summary_of_cmdres(res)
            raise CheckFailed(msg, l)
            
        msg = 'There is something wrong with contacting Github.'
        raise_CheckError_from_CommandResult(res, msg)

    
    