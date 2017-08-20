from duckietown_utils.instantiate_utils import indent
from duckietown_utils.system_cmd_imp import CmdResult
from what_the_duck.check import CheckError


def raise_CheckError_from_CommandResult(res, msg = 'Command failed'):
    assert isinstance(res, CmdResult), res
    l = summary_of_cmdres(res)
    raise CheckError(msg, l) 

def summary_of_cmdres(res):
    assert isinstance(res, CmdResult), res
    l = "The command\n  %s\n" % " ".join(res.cmd)
    l += 'returned value %r.\n' % res.ret   
    l += '\n'+ indent(res.stderr.strip(), ' stderr > ') + '\n'
    l += '\n'+ indent(res.stdout.strip(), ' stdout > ')
    return l