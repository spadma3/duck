from collections import OrderedDict
import os

from quickapp import QuickApp

import duckietown_utils as dtu
from duckietown_utils.cli import D8AppWithLogs
import rosbag

from .easy_logs_summary_imp import format_logs


__all__ = [
    'MakeVideos',
]


class MakeVideos(D8AppWithLogs, QuickApp):
    """
        Creates videos for the image topics in a log.
    """

    cmd = 'rosrun easy_logs videos'
    
    usage = """
   
Usage:

    $ %(prog)s [options]  "log query"
    
For example:

    $ %(prog)s --cloud vehicle:shamrock
    
"""
    
    def define_options(self, params):
        params.accept_extra()
        
    def define_jobs_context(self, context):
    
        extra = self.options.get_extra()
        
        if not extra:
            query = '*'
        else:
            query = extra
        
        db = self.get_easy_logs_db() 
        logs = db.query(query)
     
        self.info('Found %d logs.' % len(logs))
        logs_valid = OrderedDict()
        for log_name, log in logs.items():
            if log.valid:
                logs_valid[log_name] = log
        
        s = format_logs(logs_valid)
        self.info(s)
        
        od = self.options.output
        # if all the logs are different use those as ids
        names = [_.log_name for _ in logs_valid.values()]
        use_names = len(set(names)) == len(names)
        
        for i, (log_name, log) in enumerate(logs_valid.items()):  
            n = log.log_name if use_names else str(i)
            out = os.path.join(od, n)
                
            jobs_videos(context, log, n, out)
            
def jobs_videos(context, log, name, outd):
    filename = log.filename
    bag = rosbag.Bag(filename)
    main_camera_topic = dtu.get_image_topic(bag)
    min_messages = 3 # need at least 3 frames to make a video
    topics = [_ for _, __ in dtu.d8n_get_all_images_topic_bag(bag, min_messages=min_messages)]
    bag.close()
    
    for topic in topics:
        d = topic.replace('/','_')
        if d.startswith('_'):
            d = d[1:]
    
        out = os.path.join(outd, name + '-' + d +'.mp4') 
        j = context.comp(dtu.d8n_make_video_from_bag, filename, topic, out,
                     job_id='%s-%s' % (name, topic))
        
        # create link
        if topic == main_camera_topic:
            fn = outd + '.mp4'
            
            context.comp(link, j, out, fn)
#             dtu.write_rgb_as_jpg(grid, fn)

def link(_, src, dst):
    print('symlink %s %s' %  (src, dst))
    assert os.path.exists(src), dst

    if os.path.exists(dst):
        os.unlink(dst)  
    os.symlink(src, dst)
