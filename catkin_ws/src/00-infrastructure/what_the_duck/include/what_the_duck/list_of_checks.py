# -*- coding: utf-8 -*-
from duckietown_utils import DuckietownConstants
from duckietown_utils.constants import get_list_of_packages_in_catkin_ws
from what_the_duck.entry import SeeDocs
from what_the_duck.python_source_checks import add_python_package_checks

from .checks import *  # @UnusedWildImport
from .detect_environment import on_duckiebot
from .entry import Diagnosis, Entry


def get_checks():
    """ Returns a list of Entry """

    entries = [] #
    def add(only_run_if, desc, check, diagnosis, *suggestions):
        assert isinstance(check, Check), type(check)
        if not suggestions:
            automated = check.get_suggestion()
            if automated is not None:
                suggestions = [automated]
        E = Entry(desc=desc, check=check,
          diagnosis=diagnosis,
          resolutions=suggestions,
          only_run_if=only_run_if)
        entries.append(E)
        return E

    SSH_DIR = '~/.ssh'
    SSH_CONFIG = '~/.ssh/config'
    AUTHORIZED_KEYS = '~/.ssh/authorized_keys'
    GIT_CONFIG = '~/.gitconfig'
    JOY_DEVICE = '/dev/input/js0'

    this_is_a_duckiebot = on_duckiebot()
    this_is_a_laptop = not this_is_a_duckiebot
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


    ssh_is_there = add(None,\
        "%s exists" % SSH_DIR,
        DirExists(SSH_DIR),
        Diagnosis("SSH config dir does not exist."))

    add(ssh_is_there,
        SSH_DIR + " permissions",
        CheckPermissions(SSH_DIR, '0700'),
        Diagnosis("SSH directory has wrong permissions.")
        )

    ssh_config_exists = add(ssh_is_there,
        "%s exists" % SSH_CONFIG,
        FileExists(SSH_CONFIG),
        Diagnosis("SSH config does not exist."))

    add(None,
        "SSH option HostKeyAlgorithms is set",
        FileContains(SSH_CONFIG, "HostKeyAlgorithms ssh-rsa"),
        Diagnosis("""
        You did not follow the SSH instructions.

        The option "HostKeyAlgorithms ssh-rsa" is necessary for remote
        roslaunch to work. Otherwise it fails because of a limitation
        of the Paramiko library.

        See the discussion here:

            https://answers.ros.org/question/41446/a-is-not-in-your-ssh-known_hosts-file/

        """), Suggestion("""
        You will need to add the option, and also remove the "~/.ssh/known_hosts" file.
        (See discussion above for the why.)

        """))


    identity_file = add(ssh_config_exists,
        "Configured at least one SSH key.",
        FileContains(SSH_CONFIG, 'IdentityFile'),
        Diagnosis('You have not enabled any SSH key.'))

    add(ssh_is_there,
        "Existence of " + AUTHORIZED_KEYS,
        FileExists(AUTHORIZED_KEYS),
        Diagnosis("You did not setup the SSH authorized keys."))

    required_packages = set()

    if this_is_a_duckiebot or this_is_a_laptop:
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
            python-ruamel.yaml
            virtualenv
            libxml2-dev 
            libxslt1-dev
            libffi-dev
            bibtex2html
            pdftk
            python-frozendict
             python-tables
            
        """))

    if this_is_a_duckiebot:
        required_packages.update(make_list("""
            i2c-tools
            python-smbus
            """))

    if this_is_a_laptop:
        required_packages.update(make_list("""
            git-lfs
        """))

    # TODO
    suggested = ['emacs', 'zsh', 'nethogs']

    for p in required_packages:
        add(None, p, CheckPackageInstalled(p), Diagnosis('Package %r not installed.' % p))

    forbidden_packages = ["python-roslaunch", "rosbash"]

    for p in forbidden_packages:
        add(None, p, CheckPackageNotInstalled(p), Diagnosis('Forbidden package %r is installed.' % p))

    gitconfig = add(None,
                    "Existence of " + GIT_CONFIG,
                    FileExists(GIT_CONFIG),
                    Diagnosis("You did not do the local Git configuration."))

    add(gitconfig,
        "Git config: email",
        FileContains(GIT_CONFIG, "email ="),
        Diagnosis("You did not configure your email for Git."))

    add(gitconfig,
        "Git config: name ",
        FileContains(GIT_CONFIG, "name ="),
        Diagnosis("You did not configure your name for Git."))

    add(gitconfig,
        "Git config: push policy",
        FileContains(GIT_CONFIG, "[push]"),
        Diagnosis("You did not configure the push policy for Git."))

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

    add(None,
        'Shell is bash',
        EnvironmentVariableIsEqualTo('SHELL', '/bin/bash'),
        Diagnosis('You have not set the shell to /bin/bash'),
        Suggestion('You can change the shell using `chsh`.'))

    # check if we have internet access, and if so try github
    internet_access = add(identity_file,
                          "Working internet connection",
                          InternetConnected(),
                          Diagnosis('You are not connected to internet.'))

    add(internet_access,
        "Github configured",
        GithubLogin(),
        Diagnosis('You have not successfully setup the Github access.'))

    if this_is_a_duckiebot:
        add(None,
            "Joystick detected",
            DeviceExists(JOY_DEVICE),
            Diagnosis("The joystick is not found at %s" % JOY_DEVICE))

    DUCKIETOWN_ROOT = DuckietownConstants.DUCKIETOWN_ROOT_variable
    DUCKIEFLEET_ROOT = DuckietownConstants.DUCKIEFLEET_ROOT_variable
    DUCKIETOWN_CONFIG_SEQUENCE = DuckietownConstants.DUCKIETOWN_CONFIG_SEQUENCE_variable

    
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
        )

    scuderia_exists = add(existence[DUCKIEFLEET_ROOT],
                          'Existence of scuderia file',
                          ScuderiaFileExists(),
                          Diagnosis('You do not have a scuderia file.'),
                          SeeDocs('scuderia')
                          )

    git_lfs_installed = add(None,  # @UnusedVariable
                            'Git LFS installed',
                            GitLFSInstalled(),
                            Diagnosis('You have not installed Git LFS'),
                            SeeDocs('git-lfs'))

    ok_scuderia = add(scuderia_exists,
        'Validation of scuderia file',
        ValidScuderiaFile(),
        Diagnosis('You have an invalid scuderia file.'),
        SeeDocs('scuderia')
        )

    if this_is_a_duckiebot:
        add(scuderia_exists,
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


    machines_exists = add(ok_scuderia,
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


    add(machines_exists,
        'Machines is updated',
        MachinesNewerThanScuderia(),
        Diagnosis('Scuderia was modified after machines created'),
        )

    if True: # TODO

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
#             if package_name != 'pkg_name':
                add_python_package_checks(add, package_name, dirname)

    # TODO: DISPLAY is not set
    # files in src/ or scripts/ are executable
    # There is no file "util.py" copied from pkg_name

#     add(None,
#         'Passwordless sudo',
#         FileContains('/etc/'))

    # TODO: date
    return entries

def make_list(s):
    return [x for x in s.replace('\n', ' ').split() if x.strip()]
