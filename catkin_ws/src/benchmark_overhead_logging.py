#!/usr/bin/env python
import duckietown_utils as dtu
import sys


dtu.DuckietownConstants.show_timeit_benchmarks = True

with dtu.timeit_clock('empty 1'):
    with dtu.timeit_clock('empty 2'):
        with dtu.timeit_clock('empty 3'):
            with dtu.timeit_clock('empty 4'):
                with dtu.timeit_clock('empty 5'):
                    with dtu.timeit_clock('empty 6'):
                        with dtu.timeit_clock('empty 7'):
                            with dtu.timeit_clock('empty 8'):
                                with dtu.timeit_clock('empty 9'):
                                    pass
