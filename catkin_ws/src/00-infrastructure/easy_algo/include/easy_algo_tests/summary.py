import duckietown_utils as dtu

from duckietown_utils import logger
from easy_algo.algo_db import get_easy_algo_db
from easy_algo.formatting import format_db


@dtu.unit_test
def call_summary():
    db = get_easy_algo_db()
    s = format_db(db)
    logger.info(s)
    
    errors = []
    for f in db.family_name2config.values():
        if not f.valid:
            errors.append('Family %s: %s' % (f.family_name, f.error_if_invalid))
        for i in f.instances.values():
            if not i.valid:
                errors.append('Family %s / instance %r: %s' % (f, i.instance_name, i.error_if_invalid))
    
    if errors:
        msg ='\n' + '\n'.join(f.instances)
        raise Exception(msg)

    
if __name__ == '__main__':
    dtu.run_tests_for_this_module()
