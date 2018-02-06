import os

import duckietown_utils as dtu
from duckietown_utils.caching import get_cached
from duckietown_utils.system_cmd_imp import system_cmd_result


def detect_ipfs():
    cmd = ['ipfs', '--version']
    cwd = '.'
    try:
        _res = system_cmd_result(cwd, cmd,
                                display_stdout=False,
                                  display_stderr=False,
                                  raise_on_error=True)
    except:
        return False
    return True


def get_ipfs_hash_cached(filename):

    def f():
        return get_ipfs_hash(filename)

    basename = os.path.basename(filename)
    cache_name = 'get_ipfs_hash/' + basename
    return get_cached(cache_name, f, quiet=True)


def get_ipfs_hash(filename):
    # ipfs add --only-hash LICENSE
    #added QmcgpsyWgH8Y8ajJz1Cu72KnS5uo2Aa2LpzU7kinSupNKC LICENSE
    dtu.logger.debug('Computing IPFS hash for %s' % filename)
    cmd = ['ipfs', 'add', '--only-hash', filename]
    cwd = '.'
    res = system_cmd_result(cwd, cmd,
                            display_stdout=False,
                              display_stderr=False,
                              raise_on_error=True)

    out = res.stdout.strip().split(' ')
#    print out
    if (len(out) < 3 or out[0] != 'added' or not out[1].startswith('Qm')):
        msg = 'Invalid output for ipds:\n%s' % dtu.indent(res.stdout, ' > ')
        raise Exception(msg)
    hashed = out[1]
    return hashed
