# -*- coding: utf-8 -*-
import os

from duckietown_utils import read_package_xml_info
from duckietown_utils import DuckietownConstants
from duckietown_utils import indent
from duckietown_utils import locate_files

from .check import CheckError
from .check import CheckFailed, Check
from .entry import SeeDocs


class PythonPackageCheck(Check):

    ''' Checks that a package is well formed. '''
    def __init__(self, package_name, dirname):
        self.package_name = package_name
        self.dirname = dirname

class README(PythonPackageCheck):
    "README exists"
    def check(self):
        if self.package_name in DuckietownConstants.good_readme_exceptions:
            pass #
        else:
            check_good_readme(self.dirname, self.package_name)


class PythonPackageCheckPythonFiles(PythonPackageCheck):
    def get_all_python_files(self):
        python_files = locate_files(self.dirname, '*.py')
        return python_files

    def check(self):
        for filename in self.get_all_python_files():
            try:
                self.check_python_file(filename)
            except CheckFailed as e:
                l = 'Check failed for file:\n    %s' % filename + '\n' + str(e.long_explanation)
                raise CheckFailed(e.compact, l)

    def check_python_file(self, filename):
        raise Exception('to implement')

    def is_script(self, filename):
        """ Returns true if the file is in src/ or scripts/ or script/ """
        prev = os.path.basename(os.path.dirname(filename))
        return prev in ['src', 'scripts', 'script']

class NoBlindCopyingFromTemplate(PythonPackageCheck):
    """ Checks that the user is not blindly copying from the template """
    def check(self):
        if self.package_name in ['pkg_name']:
            return
        forbidden = [
            'include/pkg_name',
        ]
        for f in forbidden:
            ff = os.path.join(self.dirname, f)
            if os.path.exists(ff):
                msg = 'Found a file from the template (%s). This blind copying will create problems.' % f
                l = "This file is the same as in the pkg_template directory: \n"
                l += '   %s ' % ff
                raise CheckFailed(msg, l)


class NoHalfMerges(PythonPackageCheckPythonFiles):
    """ No half merges """
    def check_python_file(self, filename):
        check_no_half_merges(filename)

class NoTabs(PythonPackageCheckPythonFiles):
    """ No tabs. """
    resolution = SeeDocs('no-tabs')

    def check_python_file(self, filename):
        if DuckietownConstants.enforce_no_tabs:
            check_no_tabs(filename)

class Naming(PythonPackageCheckPythonFiles):
    """ Proper naming of Python files. """
    def check_python_file(self, filename):
        if DuckietownConstants.enforce_no_tabs:
            check_good_name(filename)

class Executable(PythonPackageCheckPythonFiles):
    """ Scripts are executable. """
    def check_python_file(self, filename):
        if self.is_script(filename):
            check_executable(filename)


class LineLengths(PythonPackageCheckPythonFiles):
    """ Maximum line lengths. """
    diagnosis = SeeDocs('max-line-length')

    def check_python_file(self, filename):

        check_line_lengths(filename, max_length=120)


class ShaBang(PythonPackageCheckPythonFiles):
    """ Sha-bang line. """
    def check_python_file(self, filename):
        if self.is_script(filename):
            check_contains_shabang(filename)

PACKAGEXML_MAX_COMMENTED_LINE = 20
CMAKELISTS_MAX_COMMENTED_LINE = 20

def is_a_template_package(package_name):
    ''' These are not checked for too many comments. '''
    return package_name in ['pkg_name', 'rostest_example']



class CheckNoCruft_packagexml(PythonPackageCheck):
    """ No templates cruft in `package.xml`. """
    def check(self):
        if is_a_template_package(self.package_name):
            return

        filename = os.path.join(self.dirname, 'package.xml')
        if not os.path.exists(filename):
            msg = 'File does not exist: %s. ' % os.path.basename(filename)
            raise CheckFailed(msg)

        data = open(filename).read()
        lines = data.split('\n')

        def is_suspicious(line):
            return '<!--' in line

        suspicious = [_ for _ in lines if is_suspicious(_)]

        ns = len(suspicious)

        if ns > PACKAGEXML_MAX_COMMENTED_LINE:
            msg = 'Too many commented out lines in "package.xml". Still have contents from template?'
            l = 'I think you just copied from the template, without deleting the things that you are not using..'
            raise CheckFailed(msg, l)

class CheckNoCruft_cmakelists(PythonPackageCheck):
    """ No templates cruft in `CMakeLists.txt`. """
    def check(self):
        if is_a_template_package(self.package_name):
            return

        filename = os.path.join(self.dirname, 'CMakeLists.txt')
        if not os.path.exists(filename):
            msg = 'File does not exist: %s. ' % os.path.basename(filename)
            raise CheckFailed(msg)

        data = open(filename).read()
        lines = data.split('\n')

        def is_suspicious(line):
            return line.strip().startswith('#')

        suspicious = [_ for _ in lines if is_suspicious(_)]

        ns = len(suspicious)

        if ns > CMAKELISTS_MAX_COMMENTED_LINE:
            msg = 'Too many commented out lines in "CMakeLists.txt". Still have contents from template?'
            l = 'I think you just copied from the template, without deleting the things that you are not using..'
            raise CheckFailed(msg, l)


class PackageXMLCheck(PythonPackageCheck):

    def check_xml(self, xml_package):  # @UnusedVariable
        msg = 'Forgot to overload check_xml().'
        raise CheckError(msg)

    def check(self):
        filename = os.path.join(self.dirname, 'package.xml')
        if not os.path.exists(filename):
            msg = 'File does not exist: %s. ' % os.path.basename(filename)
            raise CheckFailed(msg)


        import xml.etree.ElementTree as ET
        try:
            tree = ET.parse(filename)
            root = tree.getroot()
        except Exception as e:
            msg = 'Could not read "package.xml".'
            l = str(e)
            raise CheckFailed(msg, l)

        if root.tag != 'package':
            msg = 'Invalid package.xml'
            raise CheckFailed(msg)

        info = read_package_xml_info(filename)

        try:
            self.check_xml(root, info)
        except CheckFailed as e:
            msg = 'Invalid contents in "package.xml": ' + e.compact
            l = e.long_explanation
            raise CheckFailed(msg, l)


class NameIsValid(PackageXMLCheck):
    """ Valid name in "package.xml". """
    def check_xml(self, root, _info):
        # check the name is the same
        try:
            declared_package_name = get_package_name(root)
        except KeyError:
            msg = 'Could not find tag "name" in package.xml.'
            raise CheckFailed(msg)

        if declared_package_name != self.package_name:
            msg = ('Inside package.xml for %r the name is declared as %r.' %
                   (self.package_name, declared_package_name))
            raise CheckFailed(msg)
#
# class UpdateToVersion2(PackageXMLCheck):
#     """ Using version=2 format. """
#     def check_xml(self, root, _info):
#         if not 'version' in root.attrib:
#             msg = "The current ROS "


class VersionIsValid(PackageXMLCheck):
    """ Valid version in "package.xml". """
    def check_xml(self, root, _info):
        try:
            version, _attrs = get_tag_and_attributes(root, 'version')
        except KeyError:
            msg = 'Could not find tag "version" in package.xml.'
            raise CheckFailed(msg)

        if version == '0.0.0':
            msg = 'Invalid value of version %r.' % version
            raise CheckFailed(msg)

        # TODO: other checks

class LicenseIsValid(PackageXMLCheck):
    """ Valid license in "package.xml". """

    def check_xml(self, root, _info):
        # check the license is valid
        try:
            contents, _attrs = get_tag_and_attributes(root, 'license')
        except KeyError:
            msg = 'Could not find tag "license" in package.xml.'
            raise CheckFailed(msg)

        if contents in ['TODO']:
            msg = 'Invalid value of license %r.' % contents
            raise CheckFailed(msg)


class AtLeastOneAuthor(PackageXMLCheck):
    """ At least one author declared. """

    def check_xml(self, _root, info):
        if len(info.authors) == 0:
            msg = 'No authors defined.'
            l = str(info)
            raise CheckFailed(msg, l)


class AtLeastOneMaintainer(PackageXMLCheck):
    """ At least one maintainer declared. """

    def check_xml(self, _root, info):
        if len(info.maintainers) == 0:
            msg = 'No maintainers defined.'
            l = str(info)
            raise CheckFailed(msg, l)


def get_package_name(package):
    assert package.tag == 'package'
    text, _attrs = get_tag_and_attributes(package, 'name')
    return text

def get_tag_and_attributes(root, name):
    for e in root:
        if e.tag == name:
            return e.text, e.attrib
    raise KeyError(name)



def add_python_package_checks(add, package_name, dirname):
    checks = [
        README,
        NoHalfMerges,
        LicenseIsValid,
        NameIsValid,
        VersionIsValid,
#         NoTabs, # XXX
#         Naming,
        Executable,
#         ShaBang,
        NoBlindCopyingFromTemplate,
        # LineLengths,
        CheckNoCruft_packagexml,
        CheckNoCruft_cmakelists,
        AtLeastOneMaintainer,
        # AtLeastOneAuthor,
    ]
    for check in checks:
        c = check(package_name, dirname)
#         description = getattr(check, '__doc__')
        diagnosis = getattr(check, 'diagnosis', None)
        resolution = getattr(check, 'resolution', None)
        if resolution is None:
            resolutions = ()
        else:
            resolutions = (resolution,)


        what = getattr(check, '__doc__', None)
        if what is None:
            what = check.__name__
        else:
            what = what.strip()
        add(None,
            'Package %-22s: %s' % (package_name, what),
            c, diagnosis, *resolutions)

def check_contains_shabang(filename):
    contents = open(filename).read()
    shabang = '#!/usr/bin/env python'
    if not shabang in contents:
        short = os.path.basename(filename)
        msg = 'File %r does not contain the sha-bang `#!/usr/bin/env python` line.' % short
        raise CheckFailed(msg)

def check_executable(filename):
    if not os.access(filename, os.X_OK):
        short = os.path.basename(filename)
        msg = 'The file %r is not executable.' % short
        raise CheckFailed(msg)


def looks_camel_case(x):
    """ checks if there is lower UPPER sequence """
    for i in range(len(x)-1):
        a = x[i]
        b = x[i+1]
        if a.isalpha() and b.isalpha():
            if a.islower() and b.isupper():
                return True
    return False

def check_good_name(filename):
    bn = os.path.basename(filename)

    if looks_camel_case(bn):
        msg = 'Invalid filename %r uses CamelCase instead of underscored_file_names.' %bn
        raise CheckFailed(msg)


def check_no_half_merges(filename):
    contents = open(filename).read()
    if ('<' * 4 in contents) or ('>'*4 in contents):
        msg = 'It loooks like the file %r has been half-merged.' % os.path.basename(filename)
        raise CheckFailed(msg)

def check_good_readme(dirname, package_name):
    fn = os.path.join(dirname, 'README.md')
    if not os.path.exists(fn):
        msg = 'The README file for %r does not exist.' % (package_name)
        l = 'File does not exist: %s' % fn
        raise CheckFailed(msg, l)


    # Check that the first line is a H1 in the format
    #     # blah blah   {#package_name}
    contents = open(fn).read()
    lines0 = contents.split('\n')
    # remove all empty lines
    lines = [_ for _ in lines0 if _.strip()]
    first = lines[0]
    try:
        if not first.startswith('# '):
            msg = 'I expect the first line of the README to be a valid topic declaration.'
            raise ValueError(msg)
        frag = '{#%s' % package_name
        if not frag in first:
            msg = "Could not find %r in first line." % frag
            raise ValueError(msg)
    except ValueError as e:
        msg = 'README problem: ' + str(e)
        l = 'First 5 lines are:\n' + indent("\n".join(lines0[:5]), '> ')
        raise CheckFailed(msg, l)

def check_no_tabs(filename):
    # Things to check:

    # there is an "encoding" file line, and the encoding is utf-8

    contents = open(filename).read()
    if '\t' in contents:
        n = 0
        for c in contents:
            if c == '\t':
                n += 1

        short = os.path.basename(filename)
        msg = 'The file %r contains %d tab characters.' % (short, n)
        l = 'The tab characters are evil in Python code.'
        l += '\nPlease be *very* careful in changing them.'
        l += '\nDo *not* use a tool to do it (e.g. "Convert tabs to spaces"); it will get it wrong!'
        raise CheckFailed(msg, l)


def check_line_lengths(filename, max_length):
    contents = open(filename).read()
    lines = contents.split('\n')
    unacceptable = []
    maxl = max_length
    for i, line in enumerate(lines):
        if len(line) > max_length:
            unacceptable.append(i+1)
            maxl = max(maxl, len(line))
    if unacceptable:
        short = os.path.basename(filename)
        msg = ('In "%s" I found %d lines of %d chars or over, '
               'with the longest at %d chars.' %
               (short, len(unacceptable), max_length, maxl))
        l = ('The lines are %s.' % unacceptable)
        raise CheckFailed(msg, l)
