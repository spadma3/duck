
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


def format_table(rows, colspacing=1):
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
    
    sizes = []
    for i in range(len(rows[0])):
        sizes.append(max(len(row[i]) for row in rows))
    s = ''
    for row in rows:
        s += '\n'
        for size, cell in zip(sizes, row):
            s += cell.ljust(size)
            s += ' ' * colspacing
    return s

