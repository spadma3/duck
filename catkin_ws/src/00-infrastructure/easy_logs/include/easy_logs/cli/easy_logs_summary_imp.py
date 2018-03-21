from collections import defaultdict

from ruamel import yaml

import duckietown_utils as dtu
from easy_logs.resource_desc import DTR

from ..logs_db import get_easy_logs_db


def easy_logs_summary(query='*'):
    db = get_easy_logs_db()
    logs = db.query(query)
    s = format_logs(logs)
    return s


def format_logs(logs):
    if not logs:
        s = "No logs found."
        return s
    else:
        s = "Found %d logs.\n" % len(logs)

        table = get_logs_description_table(logs)
        dtu.remove_table_field(table, 'filename')
        dtu.remove_table_field(table, 'topics')
        dtu.remove_table_field(table, 'description')
#        dtu.remove_table_field(table, 'map')
        s += dtu.indent(dtu.format_table_plus(table, colspacing=4), '| ')

        counts = defaultdict(lambda:set())
        for l in logs.values():
            for rname, dtr_yaml in l.resources.items():
                counts[rname].add(dtr_yaml['name'])

        s += '\n\nCount of resources: '
        rsort = sorted(counts, key=lambda _:-len(counts[_]))
        for rname in rsort:
            rcount = len(counts[rname])
            s += '\n %3d %s' % (rcount, rname)
            if rcount <= 3:
                s += '  ' + ' '.join(counts[rname])
        return s


def get_logs_description_table(logs, color=True):
    table = []
    table.append(['#', 'Log name',
                'rc',
                  'description',
                  'hash bag',
                  'date',
                  'length',
                  'vehicle name',
                  'filename',
                  'valid',
                  'topics'])
    for i, (_, log) in enumerate(logs.items()):
        row = []
        row.append(i)
        row.append(log.log_name)
        row.append(len(log.resources))
#        row.append(log.map_name)
        row.append(log.description)
        dtr = DTR.from_yaml(log.resources['bag'])
        row.append('%s  %.1fmb\n%s' % (dtr.name, dtr.size / (1000.0 * 1000), dtr.hash['sha1']))
        row.append(log.date)
        if log.length is not None:
            l = '%5.1f s' % log.length
        else:
            l = '(none)'
        row.append(l)
        row.append(log.vehicle)
        if log.filename is None:
            row.append('not local')
        else:
            row.append(dtu.friendly_path(log.filename))
        if log.valid:
            sr = 'Yes.'
        else:
            sr = log.error_if_invalid
        row.append(sr)
        if log.bag_info is not None:
            info = yaml.dump(log.bag_info['topics'])
        else:
            info = '(none)'
        if color and not log.valid:
            row = dtu.make_row_red(row)

        row.append(info)
        table.append(row)
    return table
