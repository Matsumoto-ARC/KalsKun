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

    /* Slave からのカメラ画像を受信するスレッドの生成*/
    g_pCameraReceiver = new CameraReceiver();
    if (g_pCameraReceiver == NULL)
    {
        g_pLogger->LOG_ERROR("[masterInitialize] g_pCameraReceiver allocation failed.\n");
        goto FINISH;
    }

    /* 自身のカメラをキャプチャするスレッドの生成 */
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
        // 受信データの更新を確認
        if (receiveIndex != pShareMemory->Communicate.Index)
        {
            receiveIndex = pShareMemory->Communicate.Index;
            Mat receiveCapture = pShareMemory->Communicate.Data[receiveIndex];

            // 受信データの更新がある場合のみ、Master の持つ Camera 画像を取り込む
            captureIndex = pShareMemory->Capture.Index;
            Mat masterCapture = pShareMemory->Capture.Data[captureIndex];

            // 2つの画像からステレオマッチングを行い、物体との距離を測定する
            StereoMatching(receiveCapture, masterCapture);

        }

        // 画像更新無し時間の計測


        // ハートビート出力


        // 終了指示


    }

    retVal = ResultEnum::NormalEnd;

FINISH :

    masterFinalize();
    return retVal;
}

void StereoMatching(Mat left_img, Mat right_img) {
    /* 視差データ */
    Mat disparity_data, disparity_map;
    double min, max;

    StereoSGBM ssgbm = StereoSGBM(minDisparity, numDisparities, SADWindowSize);
    /* パラメータ設定 */
    ssgbm.speckleWindowSize = 200;
    ssgbm.speckleRange = 1;
    /* 視差マップを取得する */
    ssgbm.operator()(left_img, right_img, disparity_data);
    minMaxLoc(disparity_data, &min, &max);
    disparity_data.convertTo(disparity_map, CV_8UC3, 255 / (max - min), -255 * min / (max - min));
    /* ヒストグラムの等化 */
    equalizeHist(disparity_map, disparity_map);
}