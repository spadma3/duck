opencv_traincascade -data ~/duckietown/vehicle-detection-test/classifier \
	-featureType HAAR \
	-vec ~/duckietown/vehicle-detection-test/samples/samples.vec \
    -bg ~/duckietown/vehicle-detection-test/negatives.dat \
    -numPos 57 -numNeg 43 -numStages 10 \
    -precalcValBufSize 100 -precalcIdxBufSize 100 \
    -minHitRate 0.999 -maxFalseAlarmRate 0.5 -mode ALL \
    -w 300 -h 300
