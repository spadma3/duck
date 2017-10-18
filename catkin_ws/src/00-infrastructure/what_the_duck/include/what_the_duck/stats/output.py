import sys

from duckietown_utils import yaml_load_file
from collections import defaultdict
from bs4.element import Tag
from duckietown_utils.file_utils import write_data_to_file

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
    
    def get_sorted_hostnames(self):
        return sorted(self.hostnames)
    
    def get_sorted_test_names(self):
        return sorted(self.test_names)
    
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
    
    for test_name in sorted_test_names:
        tr = Tag(name='tr')
        td = Tag(name='td')
        td.attrs['class']= 'test_name'
#         vis = test_name.replace(' : ', )
        td.append(test_name)
        tr.append(td)
        for hostname in hostnames:
            k = test_name, hostname
            d = summary.get_data(test_name=test_name, hostname=hostname)
            if d is None:
                v = 'n/a'
                status_class = 'n-a'
            else:
                v = d['status']
                status_class = v
        
                
            td = Tag(name='td')
            td.attrs['class'] = status_class
            td.append(v)
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
    color: white;
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