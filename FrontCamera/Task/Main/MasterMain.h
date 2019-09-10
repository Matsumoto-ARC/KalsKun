#pragma once

#define minDisparity	0	/* 最小視差値 */
#define numDisparities	16	/* 最大視差値と最小視差値の差(16倍数) */
#define SADWindowSize	15	/* Sum of Absolute Differencesを計算するウィンドウのサイズ */
#define cameraDistance	15.0	/* カメラ間の距離(てきとう) */
#define cameraFocus		200.0	/* カメラの焦点距離(てきとう) */

ResultEnum masterMain(const int cameraNo);
double StereoMatching(Mat left_img, Mat right_img);