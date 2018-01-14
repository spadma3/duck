import os
    
from ruamel import yaml
import ruamel.yaml
import duckietown_utils as dtu

from easy_logs import get_easy_logs_db
import sys


def my_unicode_repr(self, data):
    return self.represent_str(data.encode('utf-8'))

#ruamel.yaml.representer.Representer.add_representer(unicode, my_unicode_repr)

def dropbox_links_main(query):
    
    base = sys.argv[1]
    output = sys.argv[2] 
    
    # dtu.get_urls_path()
    if os.path.exists(output):
        urls = yaml.load(open(output).read(), Loader=ruamel.yaml.Loader)
        for k,v in list(urls.items()):
            if not v.startswith('http'):
                del urls[k]
    else:
        urls = {}
    command = 'dropbox'
    
    files = dtu.locate_files(base, "*.bag", normalize=False)
    print('base: %s found %d' % (base, len(files)))
    for filename in files:
        logname = os.path.basename(filename)
        if logname in urls:
            dtu.logger.info('Already have %s' % logname)
            continue
        
        #filename = log.filename
        #only = filename.replace(base, '')

        only = filename
        cmd = [command, 'sharelink', only]
        res = dtu.system_cmd_result(cwd='.', cmd=cmd,
                      display_stdout=False,
                      display_stderr=True,
                      raise_on_error=True,
                      write_stdin='',
                      capture_keyboard_interrupt=False,
                      env=None)
        link = res.stdout.strip()
        link = link.replace('dl=0','dl=1')

        if 'responding' in link:
            dtu.logger.debug('Dropbox is not responding, I will stop here.')
            
            break
        
        dtu.logger.info('link : %s' % link)
        key = logname
        #key = logname.decode('utf-8')
        #print key#, key.__type__
        urls[key] = link
        url = create_hash_url(filename)
	urls[url] = link
        dtu.logger.info('url : %s' % url)
    
    yaml.default_flow_style = False
    with open(output, 'w') as f:
        yaml.dump(urls, f, default_flow_style=False, allow_unicode=True)

import urlparse
import urllib

def create_hash_url(fn):
    # scheme://netloc/path;parameters?query#fragment
    scheme = 'hash'
    netloc = 'sha1'
    path = sha1_for_file(fn)
    parameters = None
    name = os.path.basename(fn)
    size = os.path.getsize(fn)
    qs = dict(size=size,name=name)
    query = urllib.urlencode(qs)
    fragment = None
    
    url = urlparse.urlunparse((scheme, netloc, path, parameters, query, fragment))
    return url

import hashlib
def sha1_for_file(path, block_size=256*128, hr=False):
    '''
    Block size directly depends on the block size of your filesystem
    to avoid performances issues
    Here I have blocks of 4096 octets (Default NTFS)
    '''
    md5 = hashlib.sha1()
    with open(path,'rb') as f:
        for chunk in iter(lambda: f.read(block_size), b''):
             md5.update(chunk)
    return md5.hexdigest()


 
