import duckietown_utils as dtu

from what_the_duck.check import CheckError

__all__ = [
    'raise_CheckError_from_CommandResult',
    'summary_of_cmdres',
]

def raise_CheckError_from_CommandResult(res, msg = 'Command failed'):
    assert isinstance(res, dtu.CmdResult), res
    l = summary_of_cmdres(res)
    raise CheckError(msg, l) 

def summary_of_cmdres(res):
    assert isinstance(res, dtu.CmdResult), res
    l = "The command\n  %s\n" % " ".join(res.cmd)
    l += 'returned value %r.\n' % res.ret   
    l += '\n'+ dtu.indent(res.stderr.strip(), ' stderr > ') + '\n'
    l += '\n'+ dtu.indent(res.stdout.strip(), ' stdout > ')
    return l