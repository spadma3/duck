# -*- coding: utf-8 -*-
from contracts.utils import indent

from bs4 import BeautifulSoup
from bs4.element import Tag
from termcolor import colored
import cgi


class MakeSpans(object):
    def __init__(self):
        pass
    
    @staticmethod            
    def red(text): return with_style(text, 'color: red') 
    
    @staticmethod
    def green(text): return with_style(text, 'color: green')
    
    @staticmethod
    def bright_red(text): return with_style(text, 'color: red; font-weight: bold')
    
    @staticmethod
    def yellow(text): return with_style(text, 'color: orange')

def with_style(text, style):
    encoded = text # TODO: encode
    return '<span style="%s">%s</span>' % ( style, encoded)

def render_span(span):
    assert isinstance(span, Tag), span
    assert span.name == 'span'
    t = span.text.encode('utf8')
    style = span.attrs.get('style', '')
    filters = { 
        'color: red': lambda x: colored(x, 'red'),
        'color: orange': lambda x: colored(x, 'yellow'),
        'color: green': lambda x: colored(x, 'green'),
        'font-weight: bold': lambda x: colored(x,  attrs=['bold']),
    }
    for k, v in filters.items():
        if k in style:
            t = v(t)
             
    return t


def escaped_from_html(t):
    soup = my_bs(t)
    for el in list(soup.select('span')):
        e2 = render_span(el)
        el.replace_with(e2)

    for el in list(soup.select('pre')):
        dissolve(el)

    for el in list(soup.select('code')):
        dissolve(el)

    s = my_to_html_stripping_fragment(soup)
    
    s = s.replace('&gt;', '>')
    s = s.replace('&lt;', '<')
#     s = cgi.unescape(s)
    
    return s

    


def my_bs(fragment):
    """ Returns the contents wrapped in an element called "fragment".
        Expects fragment as a str in utf-8 """
    if isinstance(fragment, unicode):
        fragment = fragment.encode('utf8')
    s = '<fragment>%s</fragment>' % fragment
    
    parsed = BeautifulSoup(s, 'lxml', from_encoding='utf-8')
    res = parsed.html.body.fragment
    assert res.name == 'fragment'
    return res

    

def my_to_html_stripping_fragment(soup):
    """ Returns a string encoded in UTF-8 """
    assert soup.name == 'fragment'
    # Delete all the attrs, otherwise it is not going to look like '<fragment>'
    for k in list(soup.attrs):
        del soup.attrs[k]
    s = str(soup)
    
    S0 = '<fragment>'
    S1 = '</fragment>'
    if not s.startswith(S0):
        msg = 'Invalid generated fragment; expecting %r.' % S0
        msg += '\n\n' + indent(s, ' | ')
        raise Exception(msg) 
    
    s = s[len(S0):]
    s = s[:-len(S1)]
    return s

def dissolve(x):

    index = x.parent.index(x)
    for child in list(x.contents):
        child.extract()
        x.parent.insert(index, child)
        index += 1 

    x.extract()