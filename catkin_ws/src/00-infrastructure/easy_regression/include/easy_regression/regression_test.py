from collections import OrderedDict

from duckietown_utils.system_cmd_imp import contract


class RegressionTest():
    
    def __init__(self, logs, processors, analyzers):
        self.logs = logs
        self.processors = processors
        self.analyzers = analyzers

    @contract(returns='list(str)')
    def get_processors(self):
        return self.processors

    @contract(returns='list(str)')
    def get_analyzers(self):
        return self.analyzers
    
    def get_logs(self, algo_db):
        logs = OrderedDict()
        for s in self.logs:
            logs.update(algo_db.query(s))
        return logs