from collections import OrderedDict
import os

import duckietown_utils as dtu
from duckietown_utils.bag_visualization import count_messages_in_slice
from easy_logs.app_with_logs import D8AppWithLogs, download_if_necessary
from quickapp import QuickApp
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
        params.add_flag('all_topics',
                        help='If set, plots all topics, in addition to the camera.')
        params.accept_extra()

    def define_jobs_context(self, context):

        only_camera = not self.options.all_topics

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

#        local_db = get_easy_logs_db_fresh()
        for log_name, log in logs_valid.items():
            out = os.path.join(od, log_name)

#            present = local_db.query(log.log_name)
#            if not present:
#                print('I will have to download %s' % log.log_name)
            job_id = 'download-%s' % log.log_name
            log_downloaded = context.comp(download_if_necessary, log, job_id=job_id)
#            else:
#                log_downloaded = log

            job_id = 'setup-%s' % log_name
            context.comp_dynamic(jobs_videos, log_downloaded, log_name, out, only_camera, job_id=job_id)


def jobs_videos(context, log, name, outd, only_camera):
    assert log.filename is not None
    bag = rosbag.Bag(log.filename)
    main_camera_topic = dtu.get_image_topic(bag)
    min_messages = 5  # need at least 5 frames to make a video

    topics = [_ for _, __ in dtu.d8n_get_all_images_topic_bag(bag, min_messages=min_messages)]
    bag.close()

    only_camera_fn = outd + 'video.mp4'
    for topic in topics:
        stop_at = min_messages + 2
        actual_count, count, _stopped_early = \
            count_messages_in_slice(log.filename, topic, log.t0, log.t1, stop_at=stop_at)

        assert count >= min_messages
        if actual_count < min_messages:
            msg = 'There are only %d (out of %d) in the slice [%s, %s]' % (actual_count, count, log.t0, log.t1)
            msg += '\n topic: %s' % topic
            continue

        d = topic.replace('/', '_')
        if d.startswith('_'):
            d = d[1:]

        if only_camera:
            if topic != main_camera_topic:
                continue
            out = only_camera_fn
            j = context.comp(dtu.d8n_make_video_from_bag, log.filename, topic, out,
                             t0=log.t0, t1=log.t1,
                         job_id='%s-%s' % (name, topic))

        else:
            out = os.path.join(outd, name + '-' + d + '.mp4')
            j = context.comp(dtu.d8n_make_video_from_bag, log.filename, topic, out,
                         job_id='%s-%s' % (name, topic))

            # create link
            if topic == main_camera_topic:
                context.comp(link, j, out, only_camera_fn)


def link(_, src, dst):
    assert os.path.exists(src), dst

    if os.path.exists(dst):
        os.unlink(dst)
    os.symlink(src, dst)
