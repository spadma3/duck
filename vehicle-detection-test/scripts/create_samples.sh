#!/bin/sh

find positive_images -type f -name '*.jpg' > positives.dat
find negative_images -type f -name '*.jpg' > negatives.dat


[ -f collection.dat ] && rm -f collection.dat

for f in $(cat positives.dat); do
    im_dim=$(identify "$f" | cut -d ' ' -f 3 | tr 'x' ' ')
    echo "$f 1 0 0 $im_dim"
done > collection.dat

NUMPOS=$(wc -l collection.dat)

opencv_createsamples -info collection.dat -bgcolor 0 -bgthresh 0 \
	-vec samples/samples.vec -num $NUMPOS -w 32 -h 32


	
