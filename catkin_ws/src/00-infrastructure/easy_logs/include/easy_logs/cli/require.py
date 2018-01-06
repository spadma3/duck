from easy_logs.mis import get_log_if_not_exists

from ..logs_db import get_easy_logs_db_fresh


def require_main(log_names='*'):
    required = []
    for name in log_names:
        if not name.endswith('.bag'):
            name = name.replace('.bag', '')
        required.append(name)

    db = get_easy_logs_db_fresh()

    for name in required:
        get_log_if_not_exists(db.logs, name)

