#!/usr/bin/env python
import duckietown_utils as dtu
import sys

print('starting')

fn = sys.argv[1]

data = open(fn).read()

dtu.DuckietownConstants.show_timeit_benchmarks = True

for f in [dtu.rgb_from_jpg_by_PIL, dtu.rgb_from_jpg_by_JPEG_library, dtu.bgr_from_jpg]:
    for i in range(3):
        res = f(data)

    with dtu.timeit_clock(f.__name__):
        for i in range(10):
            res = f(data)
            print res.shape
