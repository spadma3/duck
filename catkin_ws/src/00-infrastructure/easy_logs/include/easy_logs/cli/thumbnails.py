from quickapp import QuickApp

from duckietown_utils.cli import D8AppWithLogs
from duckietown_utils.exceptions import DTUserError
import os
from duckietown_utils.bag_info import d8n_get_all_images_topic_bag
from duckietown_utils.bag_reading import BagReadProxy
from duckietown_utils.bag_logs import d8n_read_all_images_from_bag
from duckietown_utils.logging_logger import logger
from duckietown_utils.image_writing import write_image_as_jpg
from duckietown_utils.image_operations import bgr_from_rgb
from collections import OrderedDict
from easy_logs.cli.easy_logs_summary_imp import format_logs


class MakeThumbnails(D8AppWithLogs, QuickApp):
    """
        Creates thumbnails for the image topics in a log.
    """

    usage = "koepkokw "
    
    def define_options(self, params): 
        params.accept_extra()
        
    def define_jobs_context(self, context):
        extra = self.options.get_extra()
        if not extra:
            query = '*'
        else:
            if len(extra) > 1:
                msg = 'Expected only one extra argument.'
                raise DTUserError(msg)
            query = extra[0]
        
        db = self.get_easy_logs_db() 
        logs = db.query(query)
     
        self.info('Found %d logs.' % len(logs))
        logs_valid = OrderedDict()
        for log_name, log in logs.items():
            if log.valid:
                logs_valid[log_name] = log
        
        s = format_logs(logs_valid)
        print(s)
        
        out = self.options.output
        for log_name, log in logs_valid.items():
            if not log.valid:
                msg = 'Skipping %s because invalid: %s' % (log_name, log.error_if_invalid)
                logger.debug(msg) 
#             outd = os.path.join(out, log_name)
            context.comp(work, log_name, log, out, job_id=log_name)
            
def work(log_name, log, outd):
    filename = log.filename
    t0 = log.t0
    t1 = log.t1

    import rosbag  # @UnresolvedImport
    bag = rosbag.Bag(filename)
    topics = [_ for _, __ in d8n_get_all_images_topic_bag(bag)]
    bag.close()
    
    for topic in topics:
        bag = rosbag.Bag(filename)
        topics = [_ for _, __ in d8n_get_all_images_topic_bag(bag)]
        bag_proxy = BagReadProxy(bag, t0, t1)
        res = d8n_read_all_images_from_bag(bag_proxy, topic)
        
        rgb = res[0]['rgb']
        
        basename = log_name + topic.replace('/','_')
        filename = os.path.join(outd, basename+'.jpg')
        logger.info(filename)
        write_image_as_jpg(bgr_from_rgb(rgb), filename)
        print rgb.shape
        
        logger.info('done')
        bag.close()
    
        
        
        
    