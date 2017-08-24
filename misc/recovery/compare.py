#!/usr/bin/env python
from conf_tools.utils.locate_files_imp import locate_files
import os
from mcdp_utils_misc.string_utils import get_md5
from mcdp_utils_misc.memoize_simple_imp import memoize_simple
import re
basename = os.path.basename
def get_files(root):
    root_real = os.path.realpath(root)
    f0 = locate_files(root, '*')
    res = []
    for _ in f0:
#         assert os.path.exists(_), _
#         if '.py' in _: continue
#         if '.launch' in _: continue
        if not os.path.exists(_):
            print('!!! %s' % _)
        else:
            res.append(_)
    f0 = res
    f0 = map(os.path.realpath, f0)
    f0 = [_.replace(root_real + '/', '') for _ in f0]
    f0 = exclude_git(f0)
    return f0
        
class SamePlaceDifferentContent():
    def __str__(self):
        return 'changed in place'
    pass

class SamePlaceSameContent():
    
    def __str__(self):
        return 'remained same'

class Deleted():
    
    def __str__(self):
        return 'deleted'
    
class MovedAndModified():
    def __init__(self, to):
        self.to =to
    def __str__(self):
        return 'moved and changed'
    
class Unknown():
    
    def __str__(self):
        return '?'
    
class Moved():
    ''' Moved, not changed '''
    def __init__(self, to):
        self.to = to
    def __str__(self): 
        return 'moved, without changes'
      
@memoize_simple
def h(filename):
    data = open(filename).read()
    return get_md5(data)

class Hack():
    
    def __init__(self, initial, final):
        self.initial = initial
        self.final = final
       
        self.f0 = get_files(initial)
        self.f1 = get_files(final)
         
        print('Found %d files in initial repo' % len(self.f0))
        print('Found %d files in final repo' % len(self.f1))
        
              
        for _ in self.f0:
            assert os.path.exists(self.p0(_))
        for _ in self.f1:
            assert os.path.exists(self.p1(_))
            
        
        self.f1_matched = {}
        self.destiny = {}
         
        print('matching')
        for a in self.f0:
            self.destiny[a] = self.get_destiny(a)
        
        self.show_destiny()
        
    def show_destiny(self):
        for a in sorted(self.f0):
            d =  self.destiny[a]
            if not isinstance(d, Unknown):
#                 continue
                pass
            tag = str(d)
            if hasattr(d, 'to'):
                rest = '   -> %s' % d.to
            else:
                rest = ''
            print('  %-25s  %-88s  %s' % (tag, a, rest))
            
        for x in sorted(self.f1):
            if not x in self.f1_matched:
                print('CREATED   %s' % x)
    
    def matched(self, x, by):
        assert x in self.f1
        if x in self.f1_matched:
            other = self.f1_matched[x]
            msg = 'F1 file %s is matched by both:\n %s\n %s' % (x, other, by)
#             print(msg)
            raise Exception(msg)
#         assert not x in self.f1_matched
        self.f1_matched[x] = by
        
    def get_destiny(self, a):
        known_deleted = [
            'Duckietown_ROS_Guideline.md',
            'spring2016_nctu' ,
            'scuderia.yaml',
            'RPi2-Ubuntu/setup.md',
            'attic/solvv.m',
            'catkin_ws/src/f23-LED/led_emitter/CMakeLists.txt',
            'catkin_ws/src/apriltags_ros/README.md',
            'led_emitter/manifest.xml',
            'catkin_ws/src/pkg_name/howto.md',
            'catkin_ws/src/simcity/howto.md',
            'catkin_ws/src/f23-LED/README.md',
            'catkin_ws/src/simcity/howto.md',
            'setup/duckiebot_img_creation.sh',
            'setup/network_magic/fix_wifi.sh',
            'setup/network_magic/magic_commands.sh',
            'setup/network_magic/vmware_routes.sh',
            'setup/create-machines-file.py',
            'catkin_ws/src/duckietown/launch/old/apriltags_doublepipeline.launch',
            'catkin_ws/src/duckietown/launch/old/imu_remote.launch',
            'catkin_ws/src/duckietown/launch/old/lane_controller2.launch',
            'catkin_ws/src/duckietown/launch/old/lane_filter2.launch',
            'catkin_ws/src/duckietown/launch/old/lane_following.launch',
            'catkin_ws/src/duckietown/launch/old/line_detection_unit_test.launch',
            'catkin_ws/src/duckietown/launch/old/record_bag.launch',
            'catkin_ws/src/f1/anti_instagram/launch/anti_instagram_node.launch.old',
            'catkin_ws/src/isam/CATKIN_IGNORE',
            'catkin_ws/src/_attic/CATKIN_IGNORE',
            'catkin_ws/src/_attic/street_name_detector',
        ]
        for k in known_deleted:
            if k in a:
                return Deleted()
            
        moved = {
            'catkin_ws/src/f23-LED/led_detection/scripts/dp45_tests.yaml': 
            'catkin_ws/src/40-coordination/led_detection/scripts/dp45_tests.yaml',
            'catkin_ws/src/f23-LED/led_detection/scripts/all_tests.yaml':
            'catkin_ws/src/40-coordination/led_detection/scripts/all_tests.yaml',
            'catkin_ws/src/duckietown/config/baseline/line_detector/line_detector_node/default.yaml':
            'catkin_ws/src/00-infrastructure/duckietown/config/baseline/line_detector/line_detector_node/default.yaml'
        }
        if a in moved:
            to = moved[a]
            if not os.path.exists(self.p1(to)):
                raise ValueError(a, to)
            
            if contents_is_the_same(self.p0(a), self.p1(to)):
                self.matched(to, by=a)
                return Moved(to)
            else:
                self.matched(to, by=a)
                return MovedAndModified(to)
        
        
        if a in self.f1:
            if contents_is_the_same(self.p0(a), self.p1(a)):
                self.matched(a, by=a)
                return SamePlaceSameContent()
            else:
                self.matched(a, by=a)
                return SamePlaceDifferentContent()
    
        
        features = get_distinct_features(self.p0(a))

        for b in self.f1:
            common = set(features) & set(get_distinct_features(self.p1(b))) 
            if common:
                self.matched(b, by=a)
#                 print('match based on %r' % common)
                return MovedAndModified(b)
            
                
        exact = self.get_exact_match(a)
        if exact is not None:
            self.matched(exact, by=a)
            return Moved(exact)
        
        print ('could not match %s with features: %s' % (a, features))
        return Unknown()
    
    def get_exact_match(self, a):
        # don't do this with empty files
        data = open(self.p0(a)).read()
        if len(data) == 0:
            return None
        ha = h(self.p0(a))
        for b in self.f1:
            same_hash = h(self.p1(b)) == ha
            same_base = basename(b) == basename(a)
            if same_hash and same_base:
                return b
        return None
            
    def define_fate(self, filename):
        assert filename in self.f1
    def p0(self, x):
        return os.path.join(self.initial, x)
    def p1(self, x):
        return os.path.join(self.final, x)    

@memoize_simple
def get_distinct_features(a):
#     print(a)
    features = []
    if basename(a) == 'CMakeLists.txt':
        contents = open(a).read()
        m = re.search(r"project\((\w+)", contents)
        if m is not None:
            fn = 'CMakeList-project-' + m.group(1)
            features.append(fn)
    if basename(a) == 'package.xml':
        contents = open(a).read()
        m = re.search(r"<name>(\w+)</name>", contents)
        if m is not None:
            fn = 'package-xml-name-' + m.group(1)
            features.append(fn)
    if basename(a) in ['readme.md','README.md','README.txt', 'README', 'howto.md']:
        up = basename(os.path.dirname(a))
        fn = 'readme-for-%s' % up
        features.append(fn)
        
    # it it maches to three levels, it is fine
    d1 = os.path.dirname(a)    
    d2 = os.path.dirname(d1)
    fn = '%s-%s-%s' % (basename(d2), basename(d1), basename(a))
    features.append(fn)
    if basename(a) == 'setup.py':
        features.append('setup-for-%s' % basename(d1))
    
    return features

def exclude_git(fs):
    return [_ for _ in fs if not _.startswith('.git')]

def contents_is_the_same(fn1, fn2):
    return h(fn1) == h(fn2)
    
    


if __name__ == '__main__':
    h = Hack("/tmp/repo-initial", "/tmp/repo-final")
