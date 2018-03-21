import json
from urllib2 import urlopen, URLError
import duckietown_utils as dtu


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
    fields = ['loc', 'city', 'region', 'country']
    to_remove = ['phone', 'hostname', 'ip', 'org', 'postal']
    
     
    def get_invalid_response(error=None):
        data = {}
        data['error'] = error
        data['valid'] = False
        for f in fields:
            data[f] = None
        return data
    
    if not dtu.is_internet_connected():
        return get_invalid_response()
    
    
    try:
        url = 'http://ipinfo.io/json'
#         logger.debug('loading %s' % url)
        response = urlopen(url, timeout=1)
#         logger.debug('done')
        data = json.load(response)
        data['valid'] = True
        data['error'] = None
        for x in to_remove:
            if x in data:
                data.pop(x)
        return data  
    except URLError as e:
        dtu.logger.warning(str(e))
        return get_invalid_response(str(e))
        

if __name__ == '__main__': # pragma: no cover
    data = get_geolocation_data()
    print data
    print dtu.yaml_dump(data)
    