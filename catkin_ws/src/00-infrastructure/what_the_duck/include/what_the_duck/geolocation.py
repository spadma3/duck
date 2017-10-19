import json
from urllib2 import urlopen, URLError

from duckietown_utils import yaml_dump


def get_geolocation_data():
    """
        In case of success:
        
            valid = True
            error = None
            loc
            org
            city
            region
            country
            
        else:
        
            valid = False
            error = str(e)
            loc = None
            org = None
            city = None
            region = None
            country = None
        
    """
    fields = ['loc',  'city', 'region', 'country']
    to_remove = ['phone', 'hostname', 'ip', 'org', 'postal'] 
    try:
        url = 'http://ipinfo.io/json'
        response = urlopen(url)
        data = json.load(response)
        data['valid'] = True
        data['error'] = None
        for x in to_remove:
            if x in data:
                data.pop(x)
        return data  
    except URLError as e:
        data = {}
        data['error'] = str(e)
        data['valid'] = False
        for f in fields:
            data[f] = None
        return data

if __name__ == '__main__': # pragma: no cover
    data = get_geolocation_data()
    print data
    print yaml_dump(data)
    