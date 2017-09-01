import sys

from duckietown_utils import remove_table_field, format_table_plus  # @UnusedImport
from easy_algo.algo_db import get_easy_algo_db
from easy_algo.formatting import format_db, format_families, format_instances


__all__ = ['summary']

def summary():
    db = get_easy_algo_db()
    
    args = sys.argv[1:]
    if args:
        family = db.get_family(args[0])
        colorize = True
        s = format_families([family], colorize)
#         s += format_tests(family, colorize)
        s += format_instances(family, colorize) 
    else:
        s = format_db(db)
         
    print(s)
    
