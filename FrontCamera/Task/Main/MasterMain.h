#pragma once

#define minDisparity	0	/* 最小視差値 */
#define numDisparities	16	/* 最大視差値と最小視差値の差(16倍数) */
#define SADWindowSize	15	/* Sum of Absolute Differencesを計算するウィンドウのサイズ */

ResultEnum masterMain(const int cameraNo);
void StereoMatching(Mat left_img, Mat right_img);