import duckietown_utils as dtu

from compmake.utils.duration_hum import duration_compact
from what_the_duck.checks.git_repos import get_repo_age

@dtu.unit_test
def test_repo_age():
    d = dtu.get_duckietown_root()
    age_s = get_repo_age(d)
    age = duration_compact(age_s)
    print('%20s %s' % (age, d))


if __name__ == '__main__': # pragma: no cover
    dtu.run_tests_for_this_module()
