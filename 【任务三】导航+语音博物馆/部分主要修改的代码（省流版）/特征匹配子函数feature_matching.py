#! /usr/bin/ python
# -*- coding: utf-8 -*-
import cv2 as cv
import numpy as np
import os

FLANN_INDEX_KDTREE=1

class FeatureMatch():
    def __init__(self, pending_img, debug=False):
        self.__pending_img = pending_img
        self.gray_pending_img = cv.cvtColor(pending_img, cv.COLOR_BGR2GRAY)
        self.__debug = debug

    def feature_extraction(self, model_img):
        model_img = cv.imread(model_img)
        self.gray_model_img = cv.cvtColor(model_img, cv.COLOR_BGR2GRAY)
        sift = cv.xfeatures2d.SIFT_create()
        kp1, psd_des1 = sift.detectAndCompute(self.gray_model_img, None)
        kp2, psd_des2 = sift.detectAndCompute(self.gray_pending_img, None)
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(check=50)
        flann = cv.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(psd_des1, psd_des2, k=2)
        good_matches = []
        matches_points = []
        for m, n in matches:
            if m.distance < 0.5*n.distance:
                good_matches.append(m)
                matches_points.append(kp1[m.queryIdx].pt)
        good_matches = np.expand_dims(good_matches, 1)
        img_out = cv.drawMatchesKnn(model_img, kp1, self.__pending_img, kp2, good_matches, None, flags=2)
        number_of_matching = len(matches_points)
        if self.__debug == True:
            cv.imshow("debug image", img_out)
            cv.waitKey(0)
        return number_of_matching

if __name__ == "__main__":
    print(os.getcwd())
    model_image_path = "model_image/Sanxingdui_left.jpg"
    origin_image = cv.imread("model_image/Sanxingdui_left.jpg")
    fm = FeatureMatch(origin_image, debug=True)
    print(fm.feature_extraction(model_image_path))
