# -*- coding: utf-8 -*-
from duckietown_utils import DuckietownConstants, get_list_of_packages_in_catkin_ws, on_circle, on_laptop, on_duckiebot

from .checks import *  # @UnusedWildImport
from .entry import Diagnosis, Entry, SeeDocs
from .python_source_checks import add_python_package_checks
from .suite_git import add_suite_git
from .suite_ssh import good_ssh_configuration 


class Manager(object):
    def __init__(self):
        self.entries = [] 
        
    def add(self, only_run_if, desc, check, diagnosis, *suggestions):
        assert isinstance(check, Check), type(check)
        if not suggestions:
            automated = check.get_suggestion()
            if automated is not None:
                suggestions = [automated]
        E = Entry(desc=desc, check=check,
          diagnosis=diagnosis,
          resolutions=suggestions,
          only_run_if=only_run_if)
        self.entries.append(E)
        return E
        

def get_checks():
    """ Returns a list of Entry """

    manager = Manager()
    add = manager.add

    
    JOY_DEVICE = '/dev/input/js0'

    this_is_a_duckiebot = on_duckiebot()
    this_is_a_laptop = on_laptop() 
    this_is_circle = on_circle()
    
    username = getpass.getuser()

    
    if this_is_a_duckiebot:
        add(None,
            "Camera is detected",
            CommandOutputContains('sudo vcgencmd get_camera', 'detected=1'),
            Diagnosis("The camera is not connected."))

    add(None,
        "Scipy is installed",
        CanImportPackages(['scipy', 'scipy.io']),
        Diagnosis("Scipy is not installed correctly."))

    add(None,
        "sklearn is installed",
        CanImportPackages(['sklearn']),
        Diagnosis("sklearn is not installed correctly."))
    
    python_packages = [
#         'ros_node_utils',
        'procgraph',
        'comptests',
        'compmake',
        'contracts',
    ]
    for p in python_packages:
        add(None,
        "%s is installed" % p,
        CanImportPackages([p]),
        Diagnosis("Dependency %r is not installed correctly." % p),
        Suggestion(" pip install --user %s" % p))

    add(None,
        "Date is set correctly",
        CheckDate(),
        Diagnosis("The date is not set correctly."))

    not_root=add(None,
        "Not running as root",
        YouAreNotUser('root'),
        Diagnosis("You should not run the code as root."))


    if this_is_a_duckiebot:
        not_ubuntu = add(not_root,
            "Not running as ubuntu",
            YouAreNotUser('ubuntu'),
            Diagnosis("You should not run the code as ubuntu."))

        add(not_ubuntu,
            "Member of group sudo",
            UserBelongsToGroup(username, "sudo"),
            Diagnosis("You are not authorized to run sudo."))

        add(not_ubuntu,
            "Member of group input",
            UserBelongsToGroup(username, "input"),
            Diagnosis("You are not authorized to use the joystick."))

        add(not_ubuntu,
            "Member of group video",
            UserBelongsToGroup(username, "video"),
            Diagnosis("You are not authorized to read from the camera device."))

        add(not_ubuntu,
            "Member of group i2c",
            UserBelongsToGroup(username, "input"),
            Diagnosis("You are not authorized to use the motor shield."))

        for g in ['sudo','input','video','i2c']:
            add(None,
                "User ubuntu member of group `%s`" % g,
                UserBelongsToGroup("ubuntu", g),
                Diagnosis("Image not created properly."))

    if this_is_a_laptop or this_is_a_duckiebot:
        good_ssh_configuration(manager)
        
    required_packages = set()

    if this_is_a_duckiebot or this_is_a_laptop or this_is_circle:
        required_packages.update(make_list("""
            vim byobu
            git git-extras
            htop atop iftop
            aptitude apt-file
            build-essential libblas-dev liblapack-dev libatlas-base-dev gfortran libyaml-cpp-dev
            python-dev ipython python-sklearn
            python-termcolor
            ros-kinetic-desktop-full
            ntpdate

            python-pip
            ipython
            
            virtualenv
            libxml2-dev
            libxslt1-dev
            
            python-frozendict
             python-tables
            
        """))

    if this_is_a_duckiebot:
        required_packages.update(make_list("""
            i2c-tools
            python-smbus
            libffi-dev
            
            
             mplayer
             mencoder

            """))

    if this_is_a_laptop or this_is_circle:
        required_packages.update(make_list("""
            git-lfs
            pdftk
            bibtex2html
        """))

    # TODO
#     suggested = ['emacs', 'zsh', 'nethogs']

    for p in required_packages:
        add(None, "Installed APT package " + p, CheckPackageInstalled(p), Diagnosis('Package %r not installed.' % p))

    forbidden_packages = [
        "python-roslaunch", 
        "rosbash",
        'python-ruamel.yaml',
        'python-ruamel.ordereddict',
    ]

    for p in forbidden_packages:
        add(None, "You should not have installed APT package " + p, 
            CheckPackageNotInstalled(p), Diagnosis('Forbidden package %r is installed.' % p))
        
    if not this_is_circle:
        add_suite_git(manager)

    if this_is_a_duckiebot:
        add(None,
            "Edimax detected",
            CommandOutputContains('iwconfig', 'rtl8822bu'),
            Diagnosis("It seems that the Edimax is not detected."))

    add(None,
        'The hostname is configured',
        CheckHostnameConfigured(),
        Diagnosis('You have not completed a proper setup.'))

    add(None,
        '/etc/hosts is sane',
        CheckGoodHostsFile(),
        Diagnosis('The contents of /etc/hosts will cause problems later on.'))

    if this_is_a_duckiebot:
        add(None,
            'Correct kernel version',
            GoodKernel(),
            Diagnosis('You have been messing with the kernel.'),
        Suggestion('You probably need to start with a pristine SD card.'))

        add(None,
            'Wifi name configured',
            WifiNameConfigured(),
            Diagnosis('You have not completed the Wifi configuration.'))

    add(None,
        "Messages are compiled",
        CheckImportMessages(),
        Diagnosis("The messages are not compiling correctly."))

#     if not this_is_circle:
#         add(None,
#             'Shell is bash',
#             EnvironmentVariableIsEqualTo('SHELL', '/bin/bash'),
#             Diagnosis('You have not set the shell to /bin/bash'),
#             Suggestion('You can change the shell using `chsh`.'))


    if this_is_a_duckiebot:
        add(None,
            "Joystick detected",
            DeviceExists(JOY_DEVICE),
            Diagnosis("The joystick is not found at %s" % JOY_DEVICE))

    DUCKIETOWN_ROOT = DuckietownConstants.DUCKIETOWN_ROOT_variable
    DUCKIEFLEET_ROOT = DuckietownConstants.DUCKIEFLEET_ROOT_variable
    DUCKIETOWN_CONFIG_SEQUENCE = DuckietownConstants.DUCKIETOWN_CONFIG_SEQUENCE_variable


    if False:
        v = DUCKIETOWN_CONFIG_SEQUENCE
        add(None,
            'Provided environment variable %s.' % v,
            EnvironmentVariableExists(v),
            Diagnosis("%s is not set." % v),
            Suggestion('You have to set %r in your environment (e.g. .bashrc)' % v))


    variables_to_check = [DUCKIETOWN_ROOT, DUCKIEFLEET_ROOT, #DUCKIETOWN_CONFIG_SEQUENCE
                          ]

    existence = {}

    for v in variables_to_check:

        var_exists = add(None,
            'Provided environment variable %s.' % v,
            EnvironmentVariableExists(v),
            Diagnosis("%s is not set." % v),
            Suggestion('You have to set %r in your environment (e.g. .bashrc)' % v))

        existence[v] = add(var_exists,
            'Existence of path ${%s}' % v,
            DirExists('${%s}' % v),
            Diagnosis("%s is set but it points to a non-existing directory." % v)
            )


    add(existence[DUCKIETOWN_ROOT],
        'Software repo downloaded with SSH scheme.',
        GitCorrectRemote('${%s}' % DUCKIETOWN_ROOT),
        Diagnosis("You downloaded the repo using https."),
        SeeDocs('clone-software-repo')
        )
 
    # recent update
    add(existence[DUCKIETOWN_ROOT],
        'You pulled the Software repo in the last 24 hours',
        RecentlyPulled('${%s}' % DUCKIETOWN_ROOT, 24),
        Diagnosis("You did not recently pull the Software repository."),
        )

    add(existence[DUCKIEFLEET_ROOT],
        'You pulled the Duckiefleet root in the last 24 hours',
        RecentlyPulled('${%s}' % DUCKIEFLEET_ROOT, 24),
        Diagnosis("You did not recently pull the Duckiefleet repository."),
        )
 

    if not this_is_a_duckiebot:
        _git_lfs_installed = add(None,  # @UnusedVariable
                            'Git LFS installed',
                            GitLFSInstalled(),
                            Diagnosis('You have not installed Git LFS'),
                            SeeDocs('git-lfs'))

    if this_is_a_duckiebot:
        add(None,
            'This robot is mentioned in scuderia.',
            ThisRobotInScuderiaFile(),
            Diagnosis('You have not added the robot to the scuderia.'),
            SeeDocs('scuderia'))

    progs = ['roslaunch', 'rosrun']
    for prog in progs:
        add(None,
            'Good path for "%s"' % prog,
            CommandOutputContains('which %s' % prog, '/opt/ros/kinetic'),
            Diagnosis('The program `%s` is not resolved to the one in /opt/ros' % prog))

    add(None,
        'Good path for python',
        CommandOutputContains('which python', '/usr/bin/python'),
        Diagnosis('The program `python` is not resolved to the one in /usr/bin. '))
    
 

    machines_exists = add(None,
        'Existence of machines file',
        MachinesExists(),
        Diagnosis('You have an invalid or missing machines file.'),
        SeeDocs('machines'),
        )

    if this_is_a_duckiebot:
        add(machines_exists,
            'Machines file contains this robot',
            MachinesValid(),
            Diagnosis('You have an invalid  machines file.'),
            )

    found_duckiefleet = add(None, 
                            'Possible to get duckiefleet in some way',
                            FindingDuckiefleet(),
                            Diagnosis('Cannot find duckiefleet root'),
        )
    
    add(found_duckiefleet, 
        'The duckiefleet repo is up to date',
        UptodateDuckiefleet(), 
        Diagnosis('The duckiefleet repo is not up to date') )
 

    if False: # TODO

        if this_is_a_laptop:

            existence = add(None,
                'Environment variable DUCKIETOWN_DATA',
                EnvironmentVariableExists('DUCKIETOWN_DATA'),
                Diagnosis("DUCKIETOWN_DATA is not set."
    """
    The environment variable DUCKIETOWN_DATA must either:
    1) be set to "n/a"
    2) point to an existing path corresponding to Dropbox/duckietown-data.
       (containing a subdirectory 'logs')
    """
                      ))

            logs = [
                "${DUCKIETOWN_DATA}/logs/20160400-phase3-logs/dp45/20160406/20160406-226-All_red_lights_followTheLeader1-2cv.bag",
            ]
            for l in logs:
                add(existence,
                    'Log %r exists in DUCKIETOWN_DATA' % os.path.basename(l),
                    FileExists(l),
                    Diagnosis("The DUCKIETOWN_DATA folder does not contain the logs it should.")
                    )

    if False:
        # TODO: not sure if this is needed
        if this_is_a_duckiebot:
            add(None,
                'Environment variable VEHICLE_NAME',
                EnvironmentVariableExists('VEHICLE_NAME'),
                Diagnosis("""
    The environment variable VEHICLE_NAME must be the name of your robot
    (if you are on the robot)."""),
                Suggestion("""
    Add this line to ~/.bashrc:

        export VEHICLE_NAME= (your vehicle name)
    """))

    try:
        packagename2dir = get_list_of_packages_in_catkin_ws()
    except DTConfigException:
        pass
    else:
        for package_name, dirname in packagename2dir.items():
            add_python_package_checks(add, package_name, dirname)

    # TODO: DISPLAY is not set
    # files in src/ or scripts/ are executable
    # There is no file "util.py" copied from pkg_name

#     add(None,
#         'Passwordless sudo',
#         FileContains('/etc/'))

    # TODO: date
    return manager.entries

def make_list(s):
    return [x for x in s.replace('\n', ' ').split() if x.strip()]
