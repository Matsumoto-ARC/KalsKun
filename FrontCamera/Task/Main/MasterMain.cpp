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
            StereoMatching(receiveCapture, masterCapture);

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

void StereoMatching(Mat left_img, Mat right_img) {
    /* �����f�[�^ */
    Mat disparity_data, disparity_map;
    double min, max;

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
}