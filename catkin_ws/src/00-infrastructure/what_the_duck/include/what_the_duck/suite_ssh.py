# -*- coding: utf-8 -*-
from duckietown_utils import on_duckiebot

from .checks.existence import DirExists, FileExists
from .checks.file_contains import FileContains
from .checks.github import GithubLogin
from .checks.internet_connected import InternetConnected
from .checks.permissions import CheckPermissions
from .entry import Diagnosis
from .resolution import Suggestion


SSH_DIR = '~/.ssh'
SSH_CONFIG = '~/.ssh/config'
AUTHORIZED_KEYS = '~/.ssh/authorized_keys'

def good_ssh_configuration(manager):
    add = manager.add
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

    add(ssh_config_exists,
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

    if on_duckiebot():
        add(ssh_is_there,
            "Existence of " + AUTHORIZED_KEYS,
            FileExists(AUTHORIZED_KEYS),
            Diagnosis("You did not setup the SSH authorized keys."))


    # check if we have internet access, and if so try github
    internet_access = add(identity_file,
                          "Working internet connection",
                          InternetConnected(),
                          Diagnosis('You are not connected to internet.'))

    add(internet_access,
        "Github configured",
        GithubLogin(),
        Diagnosis('You have not successfully setup the Github access.'))
    
    
    