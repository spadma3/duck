from compmake.utils import duration_compact
import os
import time

from duckietown_utils.path_utils import expand_all
from what_the_duck.check import CheckError, Check, CheckFailed
from .command_output import fail_if_stdout_contains


def assert_is_git_repository(dirname, ExceptionName):
    """ Returns True if the directory exists and is a git repository. """
    assert ExceptionName in [CheckError, CheckFailed]
    d = expand_all(dirname)
    if not os.path.exists(d):
        msg = 'The repo does not exist'
        l = 'The repo does not exist in directory:\n  %s' % d
        raise ExceptionName(msg, l)

    gitd = os.path.join(d,'.git')
    if not os.path.exists(gitd):
        msg = 'The directory does not appear to be a Git repository.'
        l = 'The path %r does not exist' % gitd
        raise ExceptionName(msg)

def get_repo_age(dirname):
    fn = os.path.join(dirname, '.git', 'FETCH_HEAD')
    if not os.path.exists(fn):
        fn = dirname
    mtime = os.stat(fn).st_mtime
    now = time.time()
    diff = now-mtime
    return diff

class IsGitRepository(Check):
    def __init__(self, dirname):
        self.dirname = dirname

    def check(self):
        assert_is_git_repository(self.dirname, CheckFailed)

class RecentlyPulled(Check):
    
    """ Make sure that the directory is a git repository recently updated. """
    def __init__(self, dirname, max_age_hours):
        self.dirname = dirname
        self.max_age_hours = max_age_hours
        
    def check(self):
        d = expand_all(self.dirname)
        assert_is_git_repository(d, CheckError) # error = abort
        age = get_repo_age(d)
        max_age = self.max_age_hours * 60 * 60
        if age > max_age:
            d1 = duration_compact(age)
            d2 = duration_compact(max_age)
            msg = "The repository has not been pulled for %s (maximum tolerated %s). " % (d1,d2)
            raise CheckFailed(msg)
        
class GitCorrectRemote(Check):
    ''' Checks that the git repository has the correct remote (ssh and not https). '''
    def __init__(self, dirname):
        self.dirname = dirname

    def check(self):
        d = expand_all(self.dirname)
        if not os.path.exists(d):
            msg = 'The repo does not exist'
            l = 'The repo does not exist in directory:\n  %s' % d
            raise CheckFailed(msg, l)

        cwd = '.'
        cmd = ['git', '-C', d,  'remote', 'get-url',  'origin']
        fail_if_stdout_contains(cwd, cmd, 'https')
