import os
import shutil

from duckietown_utils import logger
from duckietown_utils.disk_hierarchy import create_tmpdir
from duckietown_utils.mkdirs import d8n_make_sure_dir_exists
from easy_algo.algo_db import get_easy_algo_db
import rosbag  # @UnresolvedImport
from duckietown_utils.bag_reading import BagReadProxy


def process_one(bag_filename, t0, t1, processors, log_out):
    logger.info('job_one()')
    logger.info('   input: %s' % bag_filename)
    logger.info('   processors: %s' % processors)
    logger.info('   out: %s' % log_out)
    
    d8n_make_sure_dir_exists(log_out)
    
    tmpdir = create_tmpdir()
    tmpfiles = []
    
    def get_tmp_bag():
        i = len(tmpfiles)
        f = os.path.join(tmpdir, 'tmp%d.bag'%i)
        tmpfiles.append(f)
        return f
        
    easy_algo_db = get_easy_algo_db()
    # instantiate processors
    processors_instances = [easy_algo_db.create_instance('processor', _) 
                            for _ in processors]
    
    tmpfiles = []
    
    try:
        for i, p in enumerate(processors_instances):
            next_bag_filename = get_tmp_bag() 
            
            in_bag = rosbag.Bag(bag_filename)

            if i == 0:
                in_bag = BagReadProxy(in_bag, t0, t1)
            
            out_bag = rosbag.Bag(next_bag_filename, 'w')
            
            logger.info('Processing:\n  in = %s\n out = %s' % 
                        (bag_filename, next_bag_filename))
            p.process_log(in_bag, out_bag)
            
            in_bag.close()
            out_bag.close()
            
            bag_filename = next_bag_filename
                
        logger.info('Creating output file %s' % log_out)
        if not processors:
            # just create symlink
            logger.info('(Just creating symlink, because there '
                        'was no processing done.)')
            os.symlink(os.path.realpath(bag_filename), log_out)
        else:
            try:
                shutil.copy(bag_filename, log_out)
            except:
                logger.error('Could not create %s' % log_out)
        logger.info('I created %s' % log_out)
            

    finally:
        for f in tmpfiles:
            logger.info(' deleting %s' % f)
            os.unlink(f)
    return log_out
