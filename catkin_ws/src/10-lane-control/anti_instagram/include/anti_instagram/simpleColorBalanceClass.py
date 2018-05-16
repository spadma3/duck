#!/usr/bin/env python
import cv2
import numpy as np
import math
import rospy
import time


"""
This class represents a basic color balance algorithm. The routine 'thresholdAnalysis' finds the two thresholds
    based on an image and a cut-off percentage. The routine 'applyTrafo' takes either two thresholds from input or the
    class own thresholds amd applies the transform.
"""
class simpleColorBalanceClass:
    # initialize
    def __init__(self):
        self.ThLow = np.zeros(3, np.int16)
        self.ThLow.fill(-1)
        self.ThHi = np.zeros(3, np.int16)
        self.ThHi.fill(-1)
        self.halfPercent = -1
        print('Instance of simpleColorBalanceClass created.')

    def apply_mask(self, matrix, mask, fill_value):
        masked = np.ma.array(matrix, mask=mask, fill_value=fill_value)
        return masked.filled()

    def apply_threshold(self, matrix, low_value, high_value):
        low_mask = matrix < low_value
        matrix = self.apply_mask(matrix, low_mask, low_value)

        high_mask = matrix > high_value
        matrix = self.apply_mask(matrix, high_mask, high_value)

        return matrix

    def thresholdAnalysis(self, img, percent):
        self.halfPercent = percent / 200.0
        channels = cv2.split(img)

        for idx, channel in enumerate(channels):
            # find the low and high precentile values (based on the input percentile)
            height, width = channel.shape
            vec_size = width * height
            flat = channel.reshape(vec_size)
            # sort entries
            flat = np.sort(flat)
            n_cols = flat.shape[0]
            # calculate thresholds
            self.ThLow[idx] = flat[int(math.floor(n_cols * self.halfPercent))]
            self.ThHi[idx] = flat[int(math.ceil(n_cols * (1.0 - self.halfPercent)))]

        return self.ThLow, self.ThHi


    def applyTrafo(self, img, ThLow = [], ThHi = []):
        if ThLow == [] and ThHi == []:
            ThLow = self.ThLow
            ThHi = self.ThHi
        # begin1=rospy.Time.now()
        channels = cv2.split(img)
        out_channels = []
        # end1=rospy.Time.now()
        # duration1=end1-begin1
        # rospy.loginfo('Splitting of image: %s' % duration1)

        for idx, channel in enumerate(channels):
            # saturate below the low percentile and above the high percentile
            # begin2=rospy.Time.now()
            thresholded = self.apply_threshold(channel, ThLow[idx], ThHi[idx])
            # end2=rospy.Time.now()
            # duration2=end2-begin2
            # rospy.loginfo('Applying threshold: %s' % duration2)
            # scale the channel
            # begin3=rospy.Time.now()
            normalized = cv2.normalize(thresholded, thresholded.copy(), 0, 255, cv2.NORM_MINMAX)
            out_channels.append(normalized)
            # end3=rospy.Time.now()
            # duration3=end3-begin3
            # rospy.loginfo('Normalize: %s' % duration3)
        return cv2.merge(out_channels)
