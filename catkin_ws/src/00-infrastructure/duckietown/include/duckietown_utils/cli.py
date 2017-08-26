from quickapp.quick_app_base import QuickAppBase
from duckietown_utils.exceptions import DTUserError
from quickapp.quick_app import QuickApp
import os

class D8App(QuickAppBase):
    
    def get_from_args_or_env(self, argname, envname):
        """ Gets either the argumnent or the environment variable."""
        options = [getattr(self.options, argname), os.environ.get(envname, None)]
        options = [_ for _ in options if _ and _.strip()]
        if not options:
            msg = ('Either provide command line argument --%s or environment variable %s.' %
                    (argname, envname))
            raise DTUserError(msg)     
        return options[0]
    
class D8AppWithJobs(D8App, QuickApp):
    pass