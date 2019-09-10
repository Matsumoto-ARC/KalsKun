#pragma once

#define minDisparity	0	/* �ŏ������l */
#define numDisparities	16	/* �ő压���l�ƍŏ������l�̍�(16�{��) */
#define SADWindowSize	15	/* Sum of Absolute Differences���v�Z����E�B���h�E�̃T�C�Y */
#define cameraDistance	15.0	/* �J�����Ԃ̋���(�Ă��Ƃ�) */
#define cameraFocus		200.0	/* �J�����̏œ_����(�Ă��Ƃ�) */

ResultEnum masterMain(const int cameraNo);
double StereoMatching(Mat left_img, Mat right_img);