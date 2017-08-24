import os
final_state="""
catkin_ws/src/00-infrastructure/duckietown
catkin_ws/src/00-infrastructure/duckietown_msgs
catkin_ws/src/00-infrastructure/easy_algo
catkin_ws/src/00-infrastructure/easy_logs
catkin_ws/src/00-infrastructure/easy_node
catkin_ws/src/00-infrastructure/what_the_duck
catkin_ws/src/10-lane-control/adafruit_drivers
catkin_ws/src/10-lane-control/anti_instagram
catkin_ws/src/10-lane-control/dagu_car
catkin_ws/src/10-lane-control/ground_projection
catkin_ws/src/10-lane-control/joy_mapper
catkin_ws/src/10-lane-control/lane_control
catkin_ws/src/10-lane-control/lane_filter
catkin_ws/src/10-lane-control/line_detector
catkin_ws/src/10-lane-control/line_detector2
catkin_ws/src/10-lane-control/pi_camera
catkin_ws/src/20-indefinite-navigation/apriltags_ros/apriltags
catkin_ws/src/20-indefinite-navigation/apriltags_ros/apriltags_ros
catkin_ws/src/20-indefinite-navigation/fsm
catkin_ws/src/20-indefinite-navigation/indefinite_navigation
catkin_ws/src/20-indefinite-navigation/intersection_control
catkin_ws/src/20-indefinite-navigation/navigation
catkin_ws/src/20-indefinite-navigation/stop_line_filter
catkin_ws/src/30-localization-and-planning/duckietown_description
catkin_ws/src/30-localization-and-planning/localization
catkin_ws/src/40-coordination/led_detection
catkin_ws/src/40-coordination/led_emitter
catkin_ws/src/40-coordination/led_interpreter
catkin_ws/src/40-coordination/led_joy_mapper
catkin_ws/src/40-coordination/rgb_led
catkin_ws/src/40-coordination/traffic_light
catkin_ws/src/50-misc-additional-functionality/mdoap
catkin_ws/src/50-misc-additional-functionality/parallel_autonomy
catkin_ws/src/50-misc-additional-functionality/vehicle_detection
catkin_ws/src/60-templates/pkg_name
catkin_ws/src/60-templates/rostest_example
catkin_ws/src/70-convenience-packages/duckie_rr_bridge
catkin_ws/src/70-convenience-packages/duckiebot_visualizer
catkin_ws/src/70-convenience-packages/duckietown_demos
catkin_ws/src/70-convenience-packages/duckietown_logs
catkin_ws/src/70-convenience-packages/duckietown_unit_test
catkin_ws/src/70-convenience-packages/veh_coordinator
catkin_ws/src/90-failed/f4-devel/bag_stamper
catkin_ws/src/90-failed/f4-devel/kinematics
catkin_ws/src/90-failed/f4-devel/visual_odometry
catkin_ws/src/90-failed/isam
catkin_ws/src/90-failed/mouse_encoder
catkin_ws/src/90-failed/simcity
catkin_ws/src/90-failed/slam
catkin_ws/src/90-failed/street_name_detector
catkin_ws/src/99-attic/adafruit_imu
catkin_ws/src/99-attic/car_supervisor
catkin_ws/src/99-attic/scene_segmentation
catkin_ws/src/99-attic/visual_odometry_line
"""

initial_state = """
catkin_ws/src/f23-LED/led_emitter
catkin_ws/src/_attic/mouse_encoder
catkin_ws/src/_attic/slam
catkin_ws/src/_attic/street_name_detector
catkin_ws/src/adafruit_drivers
catkin_ws/src/adafruit_imu
catkin_ws/src/apriltags_ros/apriltags
catkin_ws/src/apriltags_ros/apriltags_ros
catkin_ws/src/car_supervisor
catkin_ws/src/dagu_car
catkin_ws/src/duckie_rr_bridge
catkin_ws/src/duckiebot_visualizer
catkin_ws/src/duckietown
catkin_ws/src/duckietown_demos
catkin_ws/src/duckietown_description
catkin_ws/src/duckietown_logs
catkin_ws/src/duckietown_msgs
catkin_ws/src/duckietown_unit_test
catkin_ws/src/f1/anti_instagram
catkin_ws/src/f23-LED/led_detection
catkin_ws/src/f23-LED/led_interpreter
catkin_ws/src/f23-LED/led_joy_mapper
catkin_ws/src/f23-LED/rgb_led
catkin_ws/src/f23-LED/traffic_light
catkin_ws/src/f4-devel/bag_stamper
catkin_ws/src/f4-devel/kinematics
catkin_ws/src/f4-devel/visual_odometry
catkin_ws/src/fsm
catkin_ws/src/ground_projection
catkin_ws/src/indefinite_navigation
catkin_ws/src/intersection_control
catkin_ws/src/isam
catkin_ws/src/joy_mapper
catkin_ws/src/lane_control
catkin_ws/src/lane_filter
catkin_ws/src/line_detector
catkin_ws/src/localization
catkin_ws/src/mdoap
catkin_ws/src/navigation
catkin_ws/src/parallel_autonomy
catkin_ws/src/pi_camera
catkin_ws/src/pkg_name
catkin_ws/src/rostest_example
catkin_ws/src/scene_segmentation
catkin_ws/src/simcity
catkin_ws/src/stop_line_filter
catkin_ws/src/veh_coordinator
catkin_ws/src/vehicle_detection
catkin_ws/src/visual_odometry_line
"""

def load(s):
    final = [_.strip() for _ in s.split('\n') if _.strip()]
    final_name2dir = {}
    for dirname in final:
        package_name = os.path.basename(dirname)
        final_name2dir[package_name] = dirname
    return final_name2dir

initial = load(initial_state)
final = load(final_state)

print('initial: %s' % sorted(initial))
print('final: %s' % sorted(final))

for p in final:
    if not p in initial:
        print('Package %s in final is not in initial state' % p)

for p in initial:
    if not p in final:
        print('Package %s in initial is not in final state' % p)

script = """

# Check out commit

find . -name '*~HEAD*' -delete
find . -name '*~origin_andrea-*' -delete


git checkout cbca9ee028155a1eedb0b1b61470a7e747482a1b
git reset --hard cbca9ee028155a1eedb0b1b61470a7e747482a1b

git checkout -b recovery-script-4


"""

for p in initial:
    d1 = initial[p]
    d2 = final[p]
    base = os.path.dirname(d2)
    cmd = """

# move package %s
mkdir -p %s
git mv %s %s

""" % (p, base, d1, d2)
    script += cmd

script += """

rm -rf catkin_ws/src/40-coordination/led_detection/include/duckietown_utils
rm -rf catkin_ws/src/40-coordination/led_detection/include/what_the_duck
rm -rf catkin_ws/src/00-infrastructure/duckietown/launch/old

echo Next steps
echo git commit -a -m "Reorganization"
echo git merge --squash origin/andrea-config -s theirs

"""

fn = 'cmd.sh'
with open(fn, 'w') as f:
    f.write(script)

print('Written on %s' % fn)

# git merge --squash origin/andrea-config -s theirs
