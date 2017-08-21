import re

__all__ = ['indent', 'seconds_as_ms']

def indent(s, prefix, first=None):
    s = str(s)
    assert isinstance(prefix, str)
    lines = s.split('\n')
    if not lines: return ''

    if first is None:
        first= prefix

    m = max(len(prefix), len(first))

    prefix = ' ' * (m-len(prefix)) + prefix
    first = ' ' * (m-len(first)) +first

    # differnet first prefix
    res = ['%s%s' % (prefix, line.rstrip()) for line in lines]
    res[0] = '%s%s' % (first, lines[0].rstrip())
    return '\n'.join(res)


def seconds_as_ms(s):
    """ Returns a value in seconds as "XXX ms". """
    if s is None:
        return 'n/a'
    return "%.1f ms" % (s*1000)

def truncate_string_right(s, N, suff=' [..]'):
    if len(s) > N:
        s = s[:N-len(suff)] + suff
    return s

def truncate_string_left(s, N, suff='[..] '):
    if len(s) > N:
        extra = len(s) - N 
        s = suff + s[extra+len(suff):] 
    return s


escape = re.compile('\x1b\[..?m')
def remove_escapes(s):
    return escape.sub("", s)


def get_length_on_screen(s):
    """ Returns the length of s without the escapes """
    return len(remove_escapes(s))


def format_table_plus(rows, colspacing=1):
    if not rows:
        raise ValueError('Empty table.')
    nfirst = len(rows[0])
    if nfirst == 0:
        raise ValueError('Empty first row.')
    for r in rows:
        if len(r) != nfirst:
            msg = 'Row has len %s while first has length %s.' % (len(r), nfirst)
            raise ValueError(msg)
        
    # now convert all to string
    rows = [ [str(_) for _ in row] for row in rows]
    
    # for each column 
    def width_cell(s):
        return max(get_length_on_screen(x) for x in s.split('\n'))
    
    sizes = []
    for col_index in range(len(rows[0])):
        sizes.append(max(width_cell(row[col_index]) for row in rows))
        
    s = ''
    for row in rows:
        s += '\n'
        
        # how many lines do we need?
        nlines = max(num_lines(cell) for cell in row)

        for j in range(nlines):
            for size, cell in zip(sizes, row):
                cellsplit = cell.split('\n')
                if j < len(cellsplit):
                    cellj = cellsplit[j]
                else:
                    cellj = ''
                    
#                 s += '%d(' % size + cellj.ljust(size) +')'
                s +=  cellj.ljust(size) 
                s += ' ' * colspacing
            s += '\n'
    return s

def num_lines(s):
    return len(s.split('\n'))
