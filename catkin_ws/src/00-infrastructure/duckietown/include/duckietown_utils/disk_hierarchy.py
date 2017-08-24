from tempfile import mkdtemp
import yaml
import os
from duckietown_utils import logger
from duckietown_utils.exception_utils import raise_desc
from duckietown_utils.system_cmd_imp import contract


@contract(s=str, returns=str)
def dir_from_data(s):
    data = yaml.load(s)
    d = create_tmpdir()
    write_to_dir(data, d)
    return d

def write_to_dir(data, d):
    if isinstance(data, dict):
        if not os.path.exists(d):
            os.makedirs(d)
        for k, v in data.items():
            write_to_dir(v, os.path.join(d, k))
    elif isinstance(data, str):
        with open(d, 'w') as f:
            f.write(data)
        logger.info('Wrote %s' % d)
    else:
        msg = 'Invalid type.'
        raise_desc(ValueError, msg, data=data, d=d)
            

def get_mcdp_tmp_dir():
    """ Returns *the* temp dir for this project.
    Note that we need to customize with username, otherwise
    there will be permission problems.  """
    from tempfile import gettempdir
    import os
    d0 = gettempdir()
    import getpass
    username = getpass.getuser()
    d = os.path.join(d0, 'tmpdir-%s' % username)
    if not os.path.exists(d):
        try:
            os.makedirs(d)
        except OSError:
            pass
    return d

def create_tmpdir(prefix='tmpdir'):
    mcdp_tmp_dir = get_mcdp_tmp_dir()
    d = mkdtemp(dir=mcdp_tmp_dir, prefix=prefix)
    return d
