from collections import OrderedDict
from duckietown_utils.bag_info import d8n_get_all_images_topic_bag
from duckietown_utils.bag_logs import d8n_read_all_images_from_bag
from duckietown_utils.bag_reading import BagReadProxy
from duckietown_utils.cli import D8AppWithLogs
from duckietown_utils.exceptions import DTUserError
from duckietown_utils.image_composition import make_images_grid
from duckietown_utils.image_operations import bgr_from_rgb
from duckietown_utils.image_writing import write_image_as_jpg
from duckietown_utils.logging_logger import logger
import os

from quickapp import QuickApp

from easy_logs.cli.easy_logs_summary_imp import format_logs


class MakeThumbnails(D8AppWithLogs, QuickApp):
    """
        Creates thumbnails for the image topics in a log.
    """

    usage = " "
    
    def define_options(self, params):
        params.add_int('max_images', help="Max images to extract", default=20) 
        params.add_string('od', help="Output directory")
        params.accept_extra()
        
    def define_jobs_context(self, context):
        
        max_images = self.options.max_images
        
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
        
        od = self.options.od
        for i, (log_name, log) in enumerate(logs_valid.items()):  
#             outd = os.path.join(out, log_name)
            if len(logs_valid) == 1:
                out = od
            else:
                out = os.path.join(od, str(i))
                
            context.comp(work, log, out, max_images, job_id=log_name)
            
def work(log, outd, max_images):
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
        res = d8n_read_all_images_from_bag(bag_proxy, topic, max_images=max_images)
        bag.close()
        
        if len(topics) == 1:
            d0 = outd
        else:
            d = topic.replace('/','_')
            if d.startswith('_'):
                d = d[1:]
            d0 = os.path.join(outd, d)
        
        for i in range(len(res)):
            rgb = res[i]['rgb']
            filename = os.path.join(d0, ('image-%05d' % i) +'.jpg')
#             logger.info(filename)
            write_image_as_jpg(bgr_from_rgb(rgb), filename)
    
        images = [_['rgb'] for _ in res]
        grid = make_images_grid(images)
        filename = os.path.join(d0, 'grid.jpg')
        write_image_as_jpg(bgr_from_rgb(grid), filename)
            
#         logger.info('done')
        
    
        
        
        
    