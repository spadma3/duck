#!/bin/sh

find ~/duckietown/vehicle-detection-test/positive_images -type f -name '*.jpg' > ~/duckietown/vehicle-detection-test/positives.dat
find ~/duckietown/vehicle-detection-test/negative_images -type f -name '*.jpg' > ~/duckietown/vehicle-detection-test/negatives.dat


[ -f ~/duckietown/vehicle-detection-test/collection.dat ] && rm -f ~/duckietown/vehicle-detection-test/collection.dat

for f in $(cat ~/duckietown/vehicle-detection-test/positives.dat); do
    im_dim=$(identify "$f" | cut -d ' ' -f 3 | tr 'x' ' ')
    echo "$f 1 0 0 $im_dim"
done > ~/duckietown/vehicle-detection-test/collection.dat

NUMPOS=$(wc -l ~/duckietown/vehicle-detection-test/collection.dat)

[ -f ~/duckietown/vehicle-detection-test/samples/samples.vec ] && rm -f ~/duckietown/vehicle-detection-test/samples/samples.vec

opencv_createsamples -info ~/duckietown/vehicle-detection-test/collection.dat -bgcolor 0 -bgthresh 0 \
	-vec ~/duckietown/vehicle-detection-test/samples/samples.vec -num $NUMPOS -w 300 -h 300

	
