from duckietown_utils.system_cmd_imp import system_cmd_result
from what_the_duck.check import Check, CheckFailed


def is_package_installed(package_name):
    if '\n' in package_name or ' ' in package_name:
        raise ValueError(package_name)

    cmd = ['dpkg-query', '--show', "--showformat='${db:Status-Status}\n'", package_name]

    res = system_cmd_result('.', cmd,
                  display_stdout=False,
                  display_stderr=False,
                  raise_on_error=False,
                  capture_keyboard_interrupt=False,
                  env=None)
    is_installed = (res.ret == 0) and ('installed' in res.stdout)
    
    if is_installed:
        return True
    else:
        return False


class CheckPackageNotInstalled(Check):
    def __init__(self, package_name):
        self.package_name = package_name

    def check(self):
        if is_package_installed(self.package_name):
            msg = 'The package %r is installed, but it should not be.' % self.package_name
            raise CheckFailed(msg)


class CheckPackageInstalled(Check):
    def __init__(self, package_name):
        self.package_name = package_name

    def check(self):
        if not is_package_installed(self.package_name):
            msg = 'The package %r is not installed, but it should be.' % self.package_name
            raise CheckFailed(msg)
