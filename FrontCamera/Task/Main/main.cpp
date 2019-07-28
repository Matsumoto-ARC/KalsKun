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
#include "Logger/Thread/LogWriter.h"
#include "Parts/ShareMemory/ShareMemory.h"
#include "MasterMain.h"
#include "SlaveMain.h"

ResultEnum initialize(const int controllerType);
void finalize();

static LogWriter* g_pLogWriter = NULL;

int main(int argc, char* argv[])
{
    if (initialize(argc) != ResultEnum::NormalEnd)
    {
        goto FINISH;
    }

    if (argc <= 1)
    {
        masterMain();
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
        g_pLogWriter = new LogWriter((char *)"FrontCameraM");
    }
    else
    {
        g_pLogWriter = new LogWriter((char *)"FrontCameraS");
    }

    if (g_pLogWriter == NULL)
    {
        goto FINISH;
    }

    g_pLogWriter->Run();

    retVal = ResultEnum::NormalEnd;

FINISH :
    return retVal;
}

void finalize()
{
    if (g_pLogWriter != NULL)
    {
        delete g_pLogWriter;
        g_pLogWriter = NULL;
    }

    if (pShareMemory != NULL)
    {
        delete pShareMemory;
        pShareMemory = NULL;
    }
}
