#if 0
// Notify : Use OpenCV Version : 4.1.0-20190415.1
// Compile options (�\���v���p�e�B - C/C++ - �R�}���h���C�� - �ǉ��̃I�v�V����)
`pkg-config opencv --cflags` `pkg-config opencv --libs`
// Library Link (�\���v���p�e�B - �����J�[ - ���� - ���C�u�����̈ˑ��t�@�C��)
wiringPi; pthread; dl; rt;  opencv_core; opencv_video; opencv_videoio; opencv_highgui; opencv_imgproc; opencv_imgcodecs; Library
// Memo:
//  wifi �A�N�Z�X�X�L�����Fsudo iwlist wlan0 scan
#endif

#define MEMORY_MAIN

#include "Include/Common.h"
#include "Logger/Logger.h"
#include "Parts/ShareMemory/ShareMemory.h"
#include "MasterMain.h"
#include "SlaveMain.h"

ResultEnum initialize(const int controllerType);
void finalize();

int main(int argc, char* argv[])
{
    if (initialize(argc) != ResultEnum::NormalEnd)
    {
        goto FINISH;
    }

    if (argc <= 1)
    {
        int cameraNo = 0;
        if (2 <= argc)
        {
            cameraNo = atoi(argv[1]);
        }
        printf("[masterMain] CameraNo = %d\n", cameraNo);
        masterMain(cameraNo);
    }
    else
    {
        slaveMain();
    }

FINISH:
    finalize();
    return 0;
}

ResultEnum initialize(const int controllerType)
{
    ResultEnum retVal = ResultEnum::AbnormalEnd;

    wiringPiSetupSys();

    pShareMemory = new ShareMemoryStr();
    if (pShareMemory == NULL)
    {
        goto FINISH;
    }

    if (controllerType <= 1)
    {
        StartLoggerProcess((char *)"FrontCameraM");
    }
    else
    {
        StartLoggerProcess((char*)"FrontCameraS");
    }

    retVal = ResultEnum::NormalEnd;

FINISH :
    return retVal;
}

void finalize()
{
    if (pShareMemory != NULL)
    {
        delete pShareMemory;
        pShareMemory = NULL;
    }
}
