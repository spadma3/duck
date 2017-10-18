from collections import defaultdict
from compmake.utils import duration_compact
import datetime
from duckietown_utils import yaml_load_file, write_data_to_file
import sys

from bs4.element import Tag

from easy_algo import get_easy_algo_db
from what_the_duck.constant import ChecksConstants


class MongoSummary(object):
    def __init__(self):
        self.test_names = defaultdict(lambda: 0)
        self.hostnames = defaultdict(lambda: 0)
        self.th = {}
        self.host_properties = defaultdict(lambda: {})
        
    def add_scuderia(self):
        db = get_easy_algo_db()
        robots = db.query('robot', '*')
        for robot, data in robots.items():
            self.hostnames[robot] += 1
#             print data
            self.host_properties[robot]['owner'] = data.parameters['owner']
    
    def get_owner(self, hostname):
        p = self.host_properties[hostname]
        return p.get('owner', '?')
    
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
                    if v['status'] == ChecksConstants.FAIL:
                        n_not_passed += 1
#                 else:
#                     n_not_passed += 2
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
    summary.add_scuderia()
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
    td = Tag(name='td')
    td.append('hostname')
    tr.append(td)
    for hostname in hostnames:
        td = Tag(name='td')
        td.attrs['class'] = 'hostname'
        td.append(hostname)
        tr.append(td)
    table.append(tr)

    tr = Tag(name='tr')
    td = Tag(name='td')
    td.append('type')
    tr.append(td)
    for hostname in hostnames:
        td = Tag(name='td')
        stype = summary.get_host_type(hostname)
        td.attrs['class'] = stype
        td.append(stype)
        tr.append(td)
    table.append(tr)
    
    tr = Tag(name='tr')
    td = Tag(name='td')
    td.append('owner')
    tr.append(td)
    for hostname in hostnames:
        td = Tag(name='td')
        td.attrs['class'] = 'owner'
        owner = summary.get_owner(hostname) or '(no owner)'
        td.append(owner)
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

            td = Tag(name='td')
            
            s = Tag(name='span')
            
            d = summary.get_data(test_name=test_name, hostname=hostname)
            if d is None:
                td.attrs['class'] = 'n-a'
                td.append('-')
            else:
                td.attrs['class']  = d['status']
                s.attrs['title'] =d['out_long']
                upload_date = d['upload_event_date']
                elapsedTime = datetime.datetime.now() - upload_date
                when = duration_compact(elapsedTime.total_seconds())
                td.append(d['status'])
                td.append(Tag(name='br'))
                td.append('(%s)' %  when)
             
            
            td.append(s)
            tr.append(td)
        table.append(tr)
    
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
    
    width: 30em;
}

body {
font-size: 8pt;
}
 
"""
    style = Tag(name='style')
    style.append(css)
    body.append(style)
    body.append(table)
    return body

        

if __name__ == '__main__':
    filename = sys.argv[1]
    mongo_data = yaml_load_file(filename)
    html = create_summary(mongo_data)
    write_data_to_file(html, 'output.html')