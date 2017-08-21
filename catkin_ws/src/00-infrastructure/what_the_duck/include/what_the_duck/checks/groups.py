import getpass

from duckietown_utils.system_cmd_imp import CmdException, system_cmd_result
from what_the_duck.check import Check, CheckError, CheckFailed


def get_groups_in_etc(username):
    import grp
    groups = [g.gr_name for g in grp.getgrall() if username in g.gr_mem]  # @UndefinedVariable
    return groups

def get_active_groups(username):
    cmd = ['groups', username]
    try:
        res = system_cmd_result('.', cmd,
                  display_stdout=False,
                  display_stderr=False,
                  raise_on_error=True,
                  capture_keyboard_interrupt=False,
                  env=None)
    except CmdException as e:
        raise CheckError(str(e))
    active_groups = res.stdout.split() # XXX
    return active_groups


class UserBelongsToGroup(Check):
    def __init__(self, username, group):
        self.username = username
        self.group = group

    def check(self):
        configured_groups = get_groups_in_etc(self.username)
        active_groups = get_active_groups(self.username)

        if not self.group in configured_groups:
            msg = 'You are currently not a member of the group %r' % self.group
            l = """
You can correct this by adding yourself to the group, with the command:

    $ sudo adduser %s %s
    """ % (self.username, self.group)
            raise CheckFailed(msg, l)

        if not self.group in active_groups:
            msg = 'The user %r was added to group %r, but the change is not effective yet.' % (self.username, self.group)
            l = 'While you have added yourself to the group %r using `adduser`,\n' % self.group
            l += 'the change will not take effect until you close all terminals.\n'
            l += 'You can verify whether the change is effective by using the command:\n\n'
            l += '    $ groups %s' % self.username
            raise CheckFailed(msg, l)


class YouAreNotUser(Check):
    def __init__(self, username):
        self.username = username
    def check(self):
        username = getpass.getuser()
        if username == self.username:
            msg = 'You are logged in as user %s' % username
            raise CheckFailed(msg)
