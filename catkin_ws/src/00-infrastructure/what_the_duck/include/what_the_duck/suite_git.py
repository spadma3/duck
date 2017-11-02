# -*- coding: utf-8 -*-
from what_the_duck.checks.existence import FileExists
from what_the_duck.checks.file_contains import FileContains
from what_the_duck.entry import Diagnosis, SeeDocs


GIT_CONFIG = '~/.gitconfig'

def add_suite_git(manager):
    add = manager.add
    
    gitconfig = add(None,
                    "Existence of " + GIT_CONFIG,
                    FileExists(GIT_CONFIG),
                    Diagnosis("You did not do the local Git configuration."))

    add(gitconfig,
        "Git config: email",
        FileContains(GIT_CONFIG, "email ="),
        Diagnosis("You did not configure your email for Git."),
        SeeDocs('howto-git-local-config'))

    add(gitconfig,
        "Git config: name ",
        FileContains(GIT_CONFIG, "name ="),
        Diagnosis("You did not configure your name for Git."),
        SeeDocs('howto-git-local-config'))

    add(gitconfig,
        "Git config: push policy",
        FileContains(GIT_CONFIG, "[push]"),
        Diagnosis("You did not configure the push policy for Git."),
        SeeDocs('howto-git-local-config'))