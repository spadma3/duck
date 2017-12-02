from compmake.utils.duration_hum import duration_compact
from duckietown_utils.constants import get_duckietown_root

from comptests import comptest, run_module_tests
from what_the_duck.checks.git_repos import get_repo_age


@comptest
def test_repo_age():
    d = get_duckietown_root()
    age_s = get_repo_age(d) 
    age = duration_compact(age_s)
    print('%20s %s' % (age, d))


if __name__ == '__main__': # pragma: no cover
    run_module_tests()