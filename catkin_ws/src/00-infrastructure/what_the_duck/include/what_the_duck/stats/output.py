# -*- coding: utf-8 -*-
from collections import defaultdict
from compmake.utils import duration_compact
import datetime
from duckietown_utils import  write_data_to_file
from duckietown_utils.safe_pickling import safe_pickle_load
import sys

from bs4.dammit import EntitySubstitution
from bs4.element import Tag
import dateutil.parser

from contracts import contract
from easy_algo import get_easy_algo_db
from what_the_duck.constant import ChecksConstants


class MongoSummary(object):
    def __init__(self):
        self.test_names = defaultdict(lambda: 0)
        self.hostnames = defaultdict(lambda: 0)
        self.th = {}
        self.host_properties = defaultdict(lambda: {})
        self.host2last_date = {}
    
    def get_last_upload_date(self, hostname):
        return self.host2last_date.get(hostname, None)
    
    def add_scuderia(self):
        db = get_easy_algo_db()
        robots = db.query('robot', '*')
        for robot, data in robots.items():
            self.hostnames[robot] += 1
            self.host_properties[robot]['owner'] = data.parameters['owner']
    
    def get_country(self, hostname):
        p = self.host_properties[hostname]
        return p.get('country', None)
    
    def get_owner(self, hostname):
        p = self.host_properties[hostname]
        return p.get('owner', None) or p.get('username', None) or '?'
    
    @contract(mongo_data='list(dict)')
    def parse_mongo_data(self, mongo_data):
        for r in mongo_data:
            self.parse_mongo_one(r)
            
    def parse_mongo_one(self, r):
        if 'not-implemented' in r['test_name']:
            return
        
        if r.get('type', None) in ['unknown', 'cloud']:
            return
            
        hostname = r['hostname']
        self.test_names[r['test_name']] += 1
        self.hostnames[hostname] += 1
        k = r['test_name'], hostname
        
        date = r['upload_event_date']
        
        if hostname in self.host2last_date:
            previous = self.host2last_date[hostname]
            if date > previous:
                self.host2last_date[hostname] = date
        else:
            self.host2last_date[hostname] = date
        
        update = False
        if k in self.th:
            last = self.th[k]['upload_event_date']
            if date > last:
                update = True
        else:
            update = True
            
        if update:
            self.th[k] = r

            properties = ['country', 'username']
            for p in properties:
                self.host_properties[hostname][p] = r.get(p, None)
            
            
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
        return self.get_sorted_hostnames_by_date()
        
    def get_sorted_hostnames_by_date(self):
        def order(hostname):
            d = self.get_last_upload_date(hostname)
            if d is None:
                d0 = dateutil.parser.parse('Jan 1, 2016')
                return d0
            else:
                return d
        
        return sorted(self.hostnames, key=order, reverse=True)
    
    def get_sorted_hostnames_by_failures(self):
        def order(hostname):
            # sort by number of mistakes
            n_not_passed = 0
            for test_name in self.test_names:
                v = self.get_data(hostname=hostname, test_name=test_name)
                if v is not None:
                    if v['status'] == ChecksConstants.FAIL:
                        n_not_passed += 1
                    if v['status'] == ChecksConstants.OK:
                        n_not_passed += 0.01
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
#                 else:
#                     n_not_passed += 2
            return -n_not_passed
                
        return sorted(self.test_names, key=order)
    
def create_summary(mongo_data):
    summary = MongoSummary()
    summary.add_scuderia()
    summary.parse_mongo_data(mongo_data)
    body = visualize(summary)
    
    p = Tag(name='p')
    now = datetime.datetime.now()
    p.append('Report created on %s' % now.isoformat())
    body.insert(0, p)
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
        td.append(cute_stype(stype))
        tr.append(td)
    table.append(tr)
    
#     tr = Tag(name='tr')
#     td = Tag(name='td')
#     td.append('last heard')
#     tr.append(td)
#     for hostname in hostnames:
#         td = Tag(name='td')
#         date = summary.get_last_upload_date(hostname)
#         if date is not None:
#             date_s = date.isoformat()
#         else:
#             date_s = '?'
#         td.attrs['class'] = 'date'
#         td.append(date_s)
#         tr.append(td)
#     table.append(tr)
    
    
    tr = Tag(name='tr')
    td = Tag(name='td')
    td.append('owner/username')
    tr.append(td)
    for hostname in hostnames:
        td = Tag(name='td')
        td.attrs['class'] = 'owner'
        owner = summary.get_owner(hostname) or '(no owner)'
        td.append(owner)
        tr.append(td)
    table.append(tr)
    
    tr = Tag(name='tr')
    td = Tag(name='td')
    td.append('location')
    tr.append(td)
    for hostname in hostnames:
        td = Tag(name='td')
        td.attrs['class'] = 'location'
        country = summary.get_country(hostname) or ''
        td.append(cute_country(country))
        tr.append(td)
    table.append(tr)
    
    
    for test_name in sorted_test_names:
        tr = Tag(name='tr')
        td = Tag(name='td')
        td.attrs['class']= 'test_name'
#         vis = test_name.replace(' : ', )
        td.append(test_name)
        tr.append(td)
        n_failed_or_invalid = 0
        for hostname in hostnames:
            d = summary.get_data(test_name=test_name, hostname=hostname)
            if d is not None and d['status'] in [ChecksConstants.FAIL, ChecksConstants.ERROR]:
                n_failed_or_invalid += 1
        
        if n_failed_or_invalid == 0:
            continue
            
            
        if "dt_live_instagram_" in test_name or 'dt_augmented_reality_' in test_name:
            continue
        
        for hostname in hostnames:

            td = Tag(name='td')
            
            s = Tag(name='span')
            
            d = summary.get_data(test_name=test_name, hostname=hostname)
            if d is None:
                td.attrs['class'] = 'n-a'
                td.append('-')
            else:
                td.attrs['class']  = d['status']
                
                out_short = d['out_short']
                out_long = d['out_long']
#                 
#                 out_short = out_short.encode('utf8', 'ignore')
#                 out_long = out_long.encode('utf8', 'ignore')
                
#                 print out_short.__repr__()
#                 print out_long.__repr__()
                
                out = u" ||| ".join([out_short, out_long])
                td.attrs['title'] = out
                upload_date = d['upload_event_date']
                elapsedTime = datetime.datetime.now() - upload_date
                when = duration_compact(elapsedTime.total_seconds())
#                 td.append(d['status'])
#                 td.append(Tag(name='br'))
#                 td.append('(%s)' %  when)
#                 
                s.append(when)
             
            
            td.append(s)
            tr.append(td)
        table.append(tr)
    
    css = """

td.passed {
    background-color: green;
    color: white;
}


td.invalid {
    background-color: purple;
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
    max-width: 30em;
}

body {
    font-size: 8pt;
}
 
 
td {
    overflow-wrap: break-word;
    min-width: 4em;
    max-width: 4em;
    padding-bottom: 3px; 
}
td {
    border-bottom: solid 1px black;
}


"""
    style = Tag(name='style')
    style.append(css)
    body.append(style)
    body.append(table)
    
    html = Tag(name='html')
    head = Tag(name='head')
    meta = Tag(name='meta')
    meta.attrs['charset'] = 'UTF-8'
    head.append(meta)
    html.append(head)
    html.append(body) 

    return html

def cute_country(country_code):
    cute = {
#        'CA': "🇨🇦",
        'CA': "🏒",
        'CH': "🇨🇭",
        'US': "🇺🇸",
        "TW": "🐉",
    }
    return cute.get(country_code, country_code)


escaper = EntitySubstitution()
def cute_stype(x):
    cute = {
        'laptop': u"💻",
        'duckiebot': u"🚗"
    }
    s = cute.get(x, x)
    return escaper.substitute_html(s)
    
        

if __name__ == '__main__':
    filename = sys.argv[1]
    if len(sys.argv) >= 3:
        output = sys.argv[2]
    else:
        output = 'output.html'
    if filename.endswith('yaml'):
        data = open(filename).read()
        print('loading data (%d chars)' % (len(data)))
    #     from ruamel import yaml
        import yaml
    #     mongo_data = yaml.load(data, Loader=yaml.UnsafeLoader)
        mongo_data = yaml.load(data)
    else:
#         data = open(filename).read()
        mongo_data = safe_pickle_load(filename)
        
#     mongo_data = yaml_load(data)
    print('creating summary')
    html = create_summary(mongo_data)
    write_data_to_file(html, output)
    
    