#pragma once

#define minDisparity	0	/* �ŏ������l */
#define numDisparities	16	/* �ő压���l�ƍŏ������l�̍�(16�{��) */
#define SADWindowSize	15	/* Sum of Absolute Differences���v�Z����E�B���h�E�̃T�C�Y */

ResultEnum masterMain(const int cameraNo);
void StereoMatching(Mat left_img, Mat right_img);