from duckietown_utils.text_utils import remove_table_field, format_table_plus  # @UnusedImport
from easy_algo.algo_db import get_easy_algo_db
from easy_algo.formatting import format_db


__all__ = ['summary']

def summary():
    db = get_easy_algo_db()
    s = format_db(db)
    print(s)
    
