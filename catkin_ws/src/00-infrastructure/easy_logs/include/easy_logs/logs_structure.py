from collections import namedtuple


PhysicalLog = namedtuple('PhysicalLog',
     ['log_name',
     'filename',
     'map_name',
     'description',
     'vehicle',
     'date','length', 
     't0', 't1', # these are in relative time
     'size', 'bag_info',
     'has_camera',
     'valid','error_if_invalid',]) 
