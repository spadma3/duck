from collections import namedtuple, OrderedDict
import os

from bs4.element import Tag, NavigableString

import duckietown_utils as dtu
from duckietown_utils.download import get_dropbox_urls
from easy_logs.app_with_logs import D8AppWithLogs
from easy_logs.cli.easy_logs_summary_imp import get_logs_description_table

from .easy_logs_summary_imp import format_logs

__all__ = [
    'Gallery',
]


class Gallery(D8AppWithLogs):
    """
        Creates the gallery for the logs.

    """

    cmd = 'rosrun easy_logs gallery'

    def define_options(self, params):
        params.add_string('destination', help='Destination directory')
        params.accept_extra()

    def go(self):

        extra = self.options.get_extra()

        if not extra:
            query = '*'
        else:
            query = extra

        db = self.get_easy_logs_db()
        logs = db.query(query)

        logs_valid = OrderedDict()
        for log_name, log in logs.items():
            if log.valid:
                logs_valid[log_name] = log
        logs = logs_valid

        self.info('Found %d valid logs.' % len(logs))

        s = format_logs(logs)
        self.info(s)

        out = self.options.destination
        fn_html = os.path.join(out, 'index.html')

        length = 0
        for log_name, log in logs.items():
            length += log.length

        html = Tag(name='html')
        body = Tag(name='body')

        head = Tag(name='head')
        link = Tag(name='link')
        link.attrs['type'] = 'text/css'
        link.attrs['rel'] = 'stylesheet'
        link.attrs['href'] = 'style.css'
        head.append(link)
        html.append(head)
        html.append(body)

        h = Tag(name='h1')
        h.append('Duckietown Logs Database')
        body.append(h)

        c = 'Indexed %d logs, for a total length of %.1f hours.' % (len(logs), length / 3600.0)
        p = Tag(name='p')
        p.append(c)
        body.append(p)

        body.append(html_table_from_table(logs, out))

#        make_sections(body, logs, out)

        dtu.write_data_to_file(str(html), fn_html)


def html_table_from_table(logs, destination):

    res = Tag(name='table')
    tbody = Tag(name='tbody')
    thead = Tag(name='thead')
    res.append(thead)
    res.append(tbody)
    for i, (id_log, log) in enumerate(logs.items()):
        trh, tr = get_row(i, id_log, log, destination)
        tbody.append(tr)
    thead.append(trh)
    return res


def make_sections(body, logs, destination):

    for i, (id_log, log) in enumerate(logs.items()):
        section = make_section(i, id_log, log, destination)

        body.append(section)


def get_download_url(log_name):
    urls = get_dropbox_urls()
    url = urls.get(log_name + '.bag', None)
    return url


def make_section(i, id_log, log, destination):

    d = Tag(name='div')
    classes = ['log-details']

    h = Tag(name='h2')
    h.append(id_log)
    d.append(h)
    d.attrs['id'] = id_log

    rel = get_small_video(destination, id_log)
    if rel:
        video = video_for_source(rel)
        d.append((video))
    else:
        pass

    c = Tag(name='pre')

    s = []
    s.append('Vehicle: %s' % log.vehicle)
    s.append('Date: %s' % log.date)
    s.append('Length: %.1f s' % log.length)

    c.append("\n".join(s))
    d.append(c)

    rel = 'videos/%s.mp4' % id_log
    fn = os.path.join(destination, rel)
    if os.path.exists(fn):
        a = Tag(name='a')
        a.attrs['href'] = rel
        a.append('Watch video')
        p = Tag(name='p')
        p.append(a)

        d.append(a)
    else:
        msg = ('No video found for this log.')
        p = Tag(name='p')
        p.append(msg)
        d.append(p)

    url = get_download_url(log.log_name)
    if url is not None:
        a = Tag(name='a')
        a.attrs['href'] = url
        a.append('Download log')
        p = Tag(name='p')
        p.append(a)
        d.append(p)
    else:
        msg = ('No URL found for this log.')
        p = Tag(name='p')
        p.append(msg)
        d.append(p)

    rel = get_thumbnail_for_video(id_log, destination)
    if rel:
        img = Tag(name='img')
        img.attrs['class'] = 'thumbnail'
        img.attrs['src'] = rel
        d.append(img)
    else:
        msg = ('No thumbnail found for this log.')
        p = Tag(name='p')
        p.append(msg)
        d.append(p)

    if not log.valid:
        classes.append('invalid')
    else:
        classes.append('valid')

    d.attrs['class'] = " ".join(classes)

    return d


def get_thumbnail_for_video(id_log, destination):
    rel = 'thumbnails/%s.jpg' % id_log
    fn = os.path.join(destination, rel)
    if os.path.exists(fn):
        return rel
    else:
        return None


def get_row(i, id_log, log, destination):
    trh = Tag(name='tr')
    tr = Tag(name='tr')

    def td(x):

        t = Tag(name='td')
        if x is not None:
            t.append(x)
        return t

    trh.append(td('index'))
    tr.append(td(str(i)))

    trh.append(td(''))
    rel = get_small_video(destination, id_log)
    if rel:
        video = video_for_source(rel)
        tr.append(td(video))
    else:
        tr.append(td('-'))

    a = Tag(name='a')
    a.append(log.log_name)
    a.attrs['href'] = '#%s' % log.log_name

    trh.append(td('ID'))
    tr.append(td(a))

    trh.append(td('video'))

    rel = get_large_video(id_log, destination)
    if rel:
        a = Tag(name='a')
        a.attrs['href'] = rel
        a.append('video')
        tr.append(td(a))
    else:
        tr.append(td(''))

    trh.append(td('thumbnails'))

    rel = get_thumbnail_for_video(id_log, destination)
    if rel:
        a = Tag(name='a')
        a.attrs['href'] = rel
        a.append('thumbnails')
        tr.append(td(a))
    else:
        tr.append(td(''))

    trh.append(td('bag'))

    url = get_download_url(log.log_name)
    if url is not None:
        a = Tag(name='a')
        a.attrs['href'] = url
        a.append('bag')
        tr.append(td(a))
    else:
        tr.append(td(''))

    trh.append(td('date'))
    tr.append(td(log.date))

    if log.length is not None:
        l = '%5.1f s' % log.length
    else:
        l = '(none)'
    trh.append(td('length'))
    tr.append(td(l))

    trh.append(td('vehicle'))
    tr.append(td(log.vehicle))
    if log.valid:
        sr = 'Yes.'
    else:
        sr = log.error_if_invalid
    trh.append(td('valid'))
    tr.append(td(sr))
    if not log.valid:
        tr.attrs['class'] = ['invalid']
    else:
        tr.attrs['class'] = ['valid']

    return trh, tr


def video_for_source(rel):
    video = Tag(name='video')
    video.attrs['width'] = 64
    video.attrs['height'] = 48
    video.attrs['loop'] = 1
    video.attrs['autoplay'] = 1
    source = Tag(name='source')
    source.attrs['src'] = rel
    source.attrs['type'] = 'video/mp4'
    video.append(source)
    return video


def video_for_source_2(rel):
    video = Tag(name='img')
    video.attrs['width'] = 64
    video.attrs['height'] = 48
    video.attrs['src'] = rel
    return video


def get_small_video(destination, id_log):
    rel = 'small/%s.mp4' % id_log
    fn = os.path.join(destination, rel)
    if os.path.exists(fn):
        return rel
    else:
        return None


def get_large_video(id_log, destination):
    rel = 'videos/%s.mp4' % id_log
    fn = os.path.join(destination, rel)
    if os.path.exists(fn):
        return rel
    else:
        return None


GalleryEntry = namedtuple('GalleryEntry', 'log_name thumbnail video url')

