opencv_traincascade -data classifier \
	-featureType LBP \
	-vec samples/samples.vec \
    -bg negatives.dat \
    -numPos 60 -numNeg 362 -numStages 10 \
    -precalcValBufSize 1024 -precalcIdxBufSize 1024 \
    -minHitRate 0.999 -maxFalseAlarmRate 0.5 -mode ALL \
    -w 32 -h 32