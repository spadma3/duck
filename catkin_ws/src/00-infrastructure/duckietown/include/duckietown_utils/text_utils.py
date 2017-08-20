
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