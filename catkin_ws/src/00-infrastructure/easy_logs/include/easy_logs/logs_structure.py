from collections import namedtuple


PhysicalLog = namedtuple('PhysicalLog',
     ['log_name',
     'filename',
     'map_name',
     'description',
     'vehicle_name',
     'date','length',
     'size', 'bag_info',
     'has_camera',
     'valid','error_if_invalid']) 
