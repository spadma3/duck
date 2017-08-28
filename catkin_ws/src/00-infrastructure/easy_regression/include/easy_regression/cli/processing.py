import os
import shutil

from duckietown_utils import logger
from duckietown_utils.disk_hierarchy import create_tmpdir
from duckietown_utils.mkdirs import d8n_make_sure_dir_exists
from easy_algo.algo_db import get_easy_algo_db
import rosbag  # @UnresolvedImport


def process_one(bag_filename, processors, log_out):
    logger.info('job_one()')
    logger.info('   input: %s' % bag_filename)
    logger.info('   processors: %s' % processors)
    logger.info('   out: %s' % log_out)
    
    d8n_make_sure_dir_exists(log_out)
    
    def get_tmp_bag():
        d = create_tmpdir()
        return os.path.join(d, 'tmp.bag')
        
    easy_algo_db = get_easy_algo_db()
    # instantiate processors
    processors_instances = [easy_algo_db.create_instance('processor', _) 
                            for _ in processors]
    
    tmpfiles = []
    
    try:
        for p in processors_instances:
            next_bag_filename = get_tmp_bag() 
            tmpfiles.append(next_bag_filename)
            in_bag = rosbag.Bag(bag_filename)
            
            out_bag = rosbag.Bag(next_bag_filename, 'w')
            
            logger.info('Processing:\n  in = %s\n out = %s' % 
                        (bag_filename, next_bag_filename))
            p.process_log(in_bag, out_bag)
            
            in_bag.close()
            out_bag.close()
            
            os.rename(next_bag_filename, bag_filename)
    except:
        raise
    
    logger.info('Creating output file %s' % log_out)
    if not processors:
        # just create symlink
        logger.info('(Just creating symlink, because there '
                    'was no processing done.)')
        os.symlink(os.path.realpath(bag_filename), log_out)
    else:
        shutil.copy(bag_filename, log_out)
    logger.info('I created %s' % log_out)
    # TODO: delete temp file
    for t in tmpfiles:
        logger.info(' deleting %s' % t)
    
    return log_out
