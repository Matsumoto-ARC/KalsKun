#pragma once

#define minDisparity	0	/* Ε¬·l */
#define numDisparities	16	/* Εε·lΖΕ¬·lΜ·(16{) */
#define SADWindowSize	15	/* Sum of Absolute DifferencesπvZ·ιEBhEΜTCY */

ResultEnum masterMain(const int cameraNo);
void StereoMatching(Mat left_img, Mat right_img);