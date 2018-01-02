from what_the_duck.check import Check, CheckFailed
from what_the_duck.checks.fileutils import raise_CheckError_from_CommandResult,\
    summary_of_cmdres
import os
from what_the_duck.checks.command_output import fail_if_stdout_contains

import duckietown_utils as dtu

__all__ = [
    'GithubLogin',
    'GitCorrectRemote',
    'GitLFSInstalled',
]

class GithubLogin(Check):

    def check(self):

        cmd = ['ssh', '-T', 'git@github.com']

        res = dtu.system_cmd_result('.', cmd,
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


class GitLFSInstalled(Check):

    def check(self):

        cmd = ['git', 'lfs']

        res = dtu.system_cmd_result('.', cmd,
                  display_stdout=False,
                  display_stderr=False,
                  raise_on_error=False,
                  capture_keyboard_interrupt=True, # XXX?
                  env=None)

        if res.ret != 0:
            msg = '`git lfs` returned non-zero.'
            msg += '\n' + dtu.indent(res, '> ')
            raise CheckFailed(msg)

class GitCorrectRemote(Check):
    ''' Checks that the git repository has the correct remote (ssh and not https). '''
    def __init__(self, dirname):
        self.dirname = dirname

    def check(self):
        d = dtu.expand_all(self.dirname)
        if not os.path.exists(d):
            msg = 'The repo does not exist'
            l = 'The repo does not exist in directory:\n  %s' % d
            raise CheckFailed(msg, l)

        cwd = '.'
        cmd = ['git', '-C', d,  'remote', 'get-url',  'origin']
        fail_if_stdout_contains(cwd, cmd, 'https')
