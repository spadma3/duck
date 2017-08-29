from duckietown_utils.exceptions import DTConfigException
from easy_regression.conditions.references import parse_reference
from easy_regression.conditions.binary import parse_binary
from easy_regression.conditions.eval import BinaryEval, Wrapper
from easy_regression.conditions.interface import RTParseError
from duckietown_utils.exception_utils import raise_wrapped

def _parse_regression_test_check(line):
    line = line.strip()
    tokens = line.split(' ')
    if len(tokens) != 3:
        msg = 'I expect exactly 3 tokens.\nLine: "%s"\nTokens: %s' % (line, tokens)
        raise DTConfigException(msg)
    
    try:
        ref1 = parse_reference(tokens[0])
        binary = parse_binary(tokens[1])
        ref2 = parse_reference(tokens[2])
        evaluable = BinaryEval(ref1, binary, ref2)
    except RTParseError as e:
        msg = 'Cannot parse string "%s".' % line
        raise_wrapped(RTParseError, e, msg, compact=True)
    return Wrapper(evaluable)
