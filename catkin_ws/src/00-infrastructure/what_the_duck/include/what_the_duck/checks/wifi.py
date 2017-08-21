from duckietown_utils.instantiate_utils import indent
from duckietown_utils.system_cmd_imp import system_cmd_result, CmdException
from what_the_duck.check import Check, CheckError, CheckFailed


class WifiNameConfigured(Check):

    def check(self):
        initial = 'duckiebot-not-configured'

        cmd = 'iwconfig'

        try:
            res = system_cmd_result('.', cmd,
                      display_stdout=False,
                      display_stderr=False,
                      raise_on_error=True,
                      capture_keyboard_interrupt=False,
                      env=None)
        except CmdException as e:
            msg = 'The command failed\n'
            msg += '\n'+ indent(e, ' >')
            raise CheckError(msg)

        if initial in res.stdout:
            msg = 'The robot is creating a network %r.' % initial
            l = 'According to iwconfig, you are creating a network %r.' % initial
            l += '\nThis means that you have failed to properly configure the robot.'
            l += '\nIt is likely the people around you are trying to connect to your robot.'

            l += '\n\nEntire output of iwconfig:\n\n' + indent(res.stdout, '  > ')
            raise CheckFailed(msg, l)

class GoodKernel(Check):

    def check(self):
        expect = '4.4.38-v7+'
        cmd = ['uname','-r']

        try:
            res = system_cmd_result('.', cmd,
                      display_stdout=False,
                      display_stderr=False,
                      raise_on_error=True,
                      capture_keyboard_interrupt=False,
                      env=None)
        except CmdException as e:
            msg = 'The command failed\n'
            msg += '\n'+ indent(e, ' >')
            raise CheckError(msg)

        found = res.stdout.strip()
        if found != expect:
            msg = 'You are running kernel %r instead of %r.' % (found, expect)
            l = 'The kernel version is important because otherwise the Edimax\n'
            l += 'driver will not work correctly and needs to be recompiled.\n'
            l += '\n'
            l += 'Please report this error because we thought that the kernel\n'
            l += 'was prevented to update.'
            raise CheckFailed(msg, l)
