#!/usr/bin/env python
import base64
import os

from duckietown_utils import (DTUserError, DTConfigException, format_table_plus,
                              get_duckiefleet_root, indent, locate_files, memoize_simple, system_cmd_result,
                              write_data_to_file)
from duckietown_utils.cli import D8App, d8app_run
from easy_algo import get_easy_algo_db


class CreateRoster(D8App):
    """ Creates the roster """
    def define_program_options(self, params):
        params.accept_extra()
#         g = "Input/output"
#         params.add_string('image', help="Image to use.", group=g)
#         params.add_string('output', default=None, short='-o', help='Output directory', group=g) 
#         g = "Pipeline"
#         params.add_string('line_detector', default='baseline', help="Which line detector to use", group=g)
        g = 'People DB'
        params.add_string('roster', default=None, help="Create roster", 
                          group=g)

#         params.add_flag('print', help="If true, print instead of writing to file.")
    
    def go(self):
        
        db = get_easy_algo_db()
        
        extra = self.options.get_extra()
        if len(extra) > 1:
            msg = 'Only one extra param accepted.'
            raise DTUserError(msg)
        
        if len(extra) == 1:
            query = extra[0]
        else:
            query = 'all'
             
        persons = db.query_and_instance('person', query)
        
        if self.options.roster:
            roster = create_roster(persons)
            roster += roster_css
            write_data_to_file(roster, self.options.roster)
#             print(roster)
        else:
            # instance ll
            tags = set()
            for p in persons.values():
                tags.update(p.get_tags())
                
            print(list(persons))
            print('tags: %s' % list(tags))
            table = table_people(persons)
            print(table)

def table_people(people):
    table = []
    table.append(['ID', 'name', 'tags'])
    for idp, person in people.items():
        row = []
        row.append(idp)
        row.append(person.get_name())
        row.append(", ".join(person.get_tags()))
        table.append(row)
    s = ''
#     remove_table_field(table, 'filename')
    s += indent(format_table_plus(table, colspacing=4), '| ')
    return s

def create_roster(people):
    
    s = ''
    S = '\n\n'
    for k, person in people.items():
        jpg = get_image_for_person(k, 128)

        name = person.get_name()
        s += '<div id="%s" class="person">' % k 
        s += '\n <span class="name">%s</span>' % name 
        s += '\n <img src="%s"/>' % inline_jpg(jpg)
        s += '\n</div>' + S + S
    return s

nopic = 'MISSING.jpg'

def get_image_for_person(pid, size):
    basename2filename = get_images()
    b = '%s.jpg' % pid
    if not b in basename2filename:
        b = nopic
    tmp= 'tmp.jpg'
    cwd = '.'
    cmd = ['convert', basename2filename[b], '-resize', str(size), tmp]
    system_cmd_result(cwd, cmd,
                      display_stdout=False,
                      display_stderr=False,
                      raise_on_error=True) 
    jpg = open(tmp).read()
    return jpg
    
@memoize_simple
def get_images():
    found = locate_files(get_duckiefleet_root(), '*.jpg')
    basename2filename = dict((os.path.basename(_), _) for _ in found)
    if not nopic in basename2filename:
        msg = 'I expect a file %r to represent missing pictures.' % nopic
        raise DTConfigException(msg)
    return basename2filename
    
def inline_jpg(data):
    encoded = base64.b64encode(data)
    mime = 'image/jpg'
    link = 'data:%s;base64,%s' % (mime, encoded)
    return link

roster_css = """
<style>
div.person {
    display: inline-block;
    width: 12em;
    height: 12em;
    outline: solid 1px red;
    text-align: center;
    float: left;
}
div.person span.name {
    display: block;
}
div.person img {
    max-width: 10em;
    max-height: 10em;
}
</style>
"""
    
if __name__ == '__main__':
    d8app_run(CreateRoster)
    