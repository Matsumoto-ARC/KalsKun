#include <stdlib.h>
#include "Include/Common.h"
#include "Logger/Logger.h"
#include "Socket/UdpReceiver/UdpReceiver.h"
#include "Parts/ShareMemory/ShareMemory.h"
#include "Task/CameraCapture/CameraCapture.h"
#include "Task/CameraReceiver/CameraReceiver.h"
#include "MasterMain.h"

static Logger* g_pLogger = NULL;
static CameraCapture* g_pCameraCapture = NULL;
static CameraReceiver* g_pCameraReceiver = NULL;

ResultEnum masterInitialize(const int cameraNo);
void masterFinalize();

ResultEnum masterInitialize(const int cameraNo)
{
    ResultEnum retVal = ResultEnum::AbnormalEnd;

    g_pLogger = new Logger(Logger::LOG_ERROR | Logger::LOG_INFO, Logger::LogTypeEnum::BOTH_OUT);
    if (g_pLogger == NULL)
    {
        goto FINISH;
    }

    /* Slave ����̃J�����摜����M����X���b�h�̐���*/
    g_pCameraReceiver = new CameraReceiver();
    if (g_pCameraReceiver == NULL)
    {
        g_pLogger->LOG_ERROR("[masterInitialize] g_pCameraReceiver allocation failed.\n");
        goto FINISH;
    }

    /* ���g�̃J�������L���v�`������X���b�h�̐��� */
    g_pCameraCapture = new CameraCapture(cameraNo);
    if (g_pCameraCapture == NULL)
    {
        g_pLogger->LOG_ERROR("[masterInitialize] g_pCameraCapture allocation failed.\n");
        goto FINISH;
    }

    g_pCameraReceiver->Run();
    g_pCameraCapture->Run();

    retVal = ResultEnum::NormalEnd;

 FINISH :
    return retVal;
}

void masterFinalize()
{
    if (g_pCameraCapture != NULL)
    {
        g_pCameraCapture->Stop(5000);
        delete g_pCameraCapture;
        g_pCameraCapture = NULL;
    }

    if (g_pCameraReceiver != NULL)
    {
        delete g_pCameraReceiver;
        g_pCameraReceiver = NULL;
    }

    if (g_pLogger != NULL)
    {
        delete g_pLogger;
        g_pLogger = NULL;
    }
}

ResultEnum masterMain(const int cameraNo)
{
    ResultEnum retVal = ResultEnum::AbnormalEnd;
    unsigned char captureIndex = -1;
    unsigned char receiveIndex = -1;
    int key = 0;
    double distance;

    if (masterInitialize(cameraNo) != ResultEnum::NormalEnd)
    {
        goto FINISH;
    }

    g_pLogger->LOG_INFO("[masterMain] Main loop start.\n");
    while (1)
    {
        // ��M�f�[�^�̍X�V���m�F
        if (receiveIndex != pShareMemory->Communicate.Index)
        {
            receiveIndex = pShareMemory->Communicate.Index;
            Mat receiveCapture = pShareMemory->Communicate.Data[receiveIndex];

            // ��M�f�[�^�̍X�V������ꍇ�̂݁AMaster �̎��� Camera �摜����荞��
            captureIndex = pShareMemory->Capture.Index;
            Mat masterCapture = pShareMemory->Capture.Data[captureIndex];

            // 2�̉摜����X�e���I�}�b�`���O���s���A���̂Ƃ̋����𑪒肷��
            distance = StereoMatching(receiveCapture, masterCapture);

        }

        // �摜�X�V�������Ԃ̌v��


        // �n�[�g�r�[�g�o��


        // �I���w��


    }

    retVal = ResultEnum::NormalEnd;

FINISH :

    masterFinalize();
    return retVal;
}

double StereoMatching(Mat left_img, Mat right_img) {
    /* �����f�[�^ */
    Mat disparity_data, disparity_map;
    Mat cameraMatrix = (Mat_<float>(3, 3) << 
                        320, 0, 160, 
                        0, 320, 120,
                        0, 0, 1);
    Mat distCoeffs = (Mat_<float>(5, 1) << 0, 0, 0, 0, 0);
    Mat t = (Mat_<float>(3, 1) << -baseline, 0, 0);
    Mat R = Mat::eye(3, 3, CV_64F);
    Mat R1, R2, P1, P2, Q;
    Mat _3dImage;
    Mat channels[3];
    double baseline = cameraDistance;
    double z = 0.0, temp_z;
    double min, max;

    Q = makeQMatrix(Point2d((left_img.cols - 1.0) / 2.0, (left_img.rows - 1.0) / 2.0), cameraFocus, baseline);
    // stereoRectify(camareMatrix, distCoeffs, cameraMatrix, distCoeffs, Size(320, 240), R, t, R1, R2, P1, P2, Q);

    StereoSGBM ssgbm = StereoSGBM(minDisparity, numDisparities, SADWindowSize);
    /* �p�����[�^�ݒ� */
    ssgbm.speckleWindowSize = 200;
    ssgbm.speckleRange = 1;
    /* �����}�b�v���擾���� */
    ssgbm.operator()(left_img, right_img, disparity_data);
    minMaxLoc(disparity_data, &min, &max);
    disparity_data.convertTo(disparity_map, CV_8UC3, 255 / (max - min), -255 * min / (max - min));
    /* �q�X�g�O�����̓��� */
    equalizeHist(disparity_map, disparity_map);

    reprojectImageTo3D(disparity_map, _3dImage, Q);
    split(_3dImage, channels);

    for (int j = 0; j < _3dImage.rows; j++) {
        for(int i = 0; i < _3dImage.cols; i++) {
            /* temp_x = channels[0].at<float>(j, i); */
            /* temp_y = channels[1].at<float>(j, i); */
            temp_z = channels[3].at<float>(j, i);
            /* z�������l�̏ꍇ�A�l���X�V���� */
            if ((z == 0) && (temp_z > 0)) {
                z = temp_z;
            /* temp_z�̕����������ꍇ�A�l���X�V���� */
            } else if((z > temp_z) && (temp_z > 0)) {
                z = temp_z;     /* ��ԋ߂����� */
            }
        }
    }

    return z;
}

Mat makeQMatrix(Point2d image_center,double focal_length, double baseline)
{
    Mat Q=Mat::eye(4,4,CV_64F);
    Q.at<double>(0,3)=-image_center.x;
    Q.at<double>(1,3)=-image_center.y;
    Q.at<double>(2,3)=focal_length;
    Q.at<double>(3,3)=0.0;
    Q.at<double>(2,2)=0.0;
    Q.at<double>(3,2)=1.0/baseline;
 
    return Q;
}