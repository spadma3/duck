import sys

from duckietown_utils import yaml_load_file
from collections import defaultdict
from bs4.element import Tag
from duckietown_utils.file_utils import write_data_to_file
from what_the_duck.constant import ChecksConstants
from compmake.utils.duration_hum import duration_compact
import datetime

class MongoSummary(object):
    def __init__(self):
        self.test_names = defaultdict(lambda: 0)
        self.hostnames = defaultdict(lambda: 0)
        self.th = {}
        
    def parse_mongo_data(self, mongo_data):
        for r in mongo_data:
            self.parse_mongo_one(r)
            
    def parse_mongo_one(self, r):
        if 'not-implemented' in r['test_name']:
            return
        self.test_names[r['test_name']] += 1
        self.hostnames[r['hostname']] += 1
        k = r['test_name'], r['hostname']
        date = r['upload_event_date']
        if k in self.th:
            last = self.th[k]['upload_event_date']
            if date > last:
                self.th[k] = r
        else:
            self.th[k] = r
            
    def get_data(self, hostname, test_name):
        """ Returns None if does not exist """
        k = test_name, hostname
        return self.th.get(k, None)
    
    def get_host_type(self, hostname):
        for test_name in self.test_names:
            v = self.get_data(hostname=hostname, test_name=test_name)
            if v is not None:
                return v.get('type', '???')
        return '?'
            
        
    def get_sorted_hostnames(self):
        def order(hostname):
            # sort by number of mistakes
            n_not_passed = 0
            for test_name in self.test_names:
                v = self.get_data(hostname=hostname, test_name=test_name)
                if v is not None:
                    if v['status'] != ChecksConstants.OK:
                        n_not_passed += 1
                else:
                    n_not_passed += 2
            return -n_not_passed
                
        return sorted(self.hostnames, key=order)
    
    def get_sorted_test_names(self):
        def order(test_name):
            # sort by number of mistakes
            n_not_passed = 0
            for hostname in self.hostnames:
                v = self.get_data(hostname=hostname, test_name=test_name)
                if v is not None:
                    if v['status'] != ChecksConstants.OK:
                        n_not_passed += 1
                else:
                    n_not_passed += 2
            return -n_not_passed
                
        return sorted(self.test_names, key=order)
    
def create_summary(mongo_data):
    summary = MongoSummary()
    summary.parse_mongo_data(mongo_data)
    body = visualize(summary)
    html = str(body)
    return html
     


#     status: passed
# username: dzenan
# hostname: dzenanThinkPad13
# out_long: ''
# what_the_duck_version: '1.1'
# upload_event_date: 2017-10-18 16:06:31.178000
# test_name: 'Package navigation            : At least one maintainer declared.'
# out_short: ''
# upload_event_id: dzenanThinkPad13-2017-10-18_16:06:31.178162


def visualize(summary):
    hostnames = summary.get_sorted_hostnames()
    sorted_test_names = summary.get_sorted_test_names()
    
    body = Tag(name='body')
    table = Tag(name='table')
    
    tr = Tag(name='tr')
    tr.append(Tag(name='td'))
    for hostname in hostnames:
        td = Tag(name='td')
        td.attrs['class'] = 'hostname'
        td.append(hostname)
        tr.append(td)
    table.append(tr)

    tr = Tag(name='tr')
    tr.append(Tag(name='td'))
    for hostname in hostnames:
        td = Tag(name='td')
        stype = summary.get_host_type(hostname)
        td.attrs['class'] = stype
        td.append(stype)
        tr.append(td)
    table.append(tr)
    
    for test_name in sorted_test_names:
        tr = Tag(name='tr')
        td = Tag(name='td')
        td.attrs['class']= 'test_name'
#         vis = test_name.replace(' : ', )
        td.append(test_name)
        tr.append(td)
        for hostname in hostnames:

            d = summary.get_data(test_name=test_name, hostname=hostname)
            if d is None:
                v = 'n/a'
                status_class = 'n-a'
                when = 'n/a'
                extra = ''
            else:
                v = d['status']
                status_class = v
                extra = d['out_long']
                upload_date = d['upload_event_date']
                elapsedTime = datetime.datetime.now() - upload_date
                when = duration_compact(elapsedTime.total_seconds())
            td = Tag(name='td')
            td.attrs['class'] = status_class
            s = Tag(name='span')
            s.attrs['title'] = extra
            s.append('%s (%s)' % (v, when))
            td.append(s)
            tr.append(td)
        table.append(tr)
    body.append(table)
    css = """

td.passed {
    background-color: green;
    color: white;
}


td.skipped {
    background-color: yellow;
    color: black;
}

td.failed {
    background-color: red;
    color: white;
}
td.hostname {
    font-family: monospace;
}
td.test_name {
    font-size:smaller;
    width: 30em;
}
 
"""
    style = Tag(name='style')
    style.append(css)
    body.append(style)
        
    return body

        

if __name__ == '__main__':
    filename = sys.argv[1]
    mongo_data = yaml_load_file(filename)
    html = create_summary(mongo_data)
    write_data_to_file(html, 'output.html')