from duckietown_utils.mkdirs import d8n_make_sure_dir_exists
from duckietown_utils.friendly_path_imp import friendly_path
from duckietown_utils import logger

def write_data_to_file(data, filename):
    """ Writes the data to the given filename. """
    if not isinstance(data, str):
        msg = 'Expected "data" to be a string, not %s.' % type(data).__name__
        raise ValueError(msg)
    if len(filename) > 256:
        msg = 'Invalid argument filename: too long. Did you confuse it with data?'
        raise ValueError(msg)
    d8n_make_sure_dir_exists(filename) 
    with open(filename, 'w') as f:
        f.write(data)
    logger.debug('Written to: %s' % friendly_path(filename))
     
    