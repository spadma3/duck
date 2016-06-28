opencv_traincascade -data ~/duckietown/vehicle-detection-test/classifier \
	-featureType LBP \
	-vec ~/duckietown/vehicle-detection-test/samples/samples.vec \
    -bg ~/duckietown/vehicle-detection-test/negatives.dat \
    -numPos 57 -numNeg 8 -numStages 20 \
    -precalcValBufSize 500 -precalcIdxBufSize 500 \
    -minHitRate 0.999 -maxFalseAlarmRate 0.5 -mode ALL \
    -w 300 -h 300