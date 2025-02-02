#include <stdio.h>
#include <stdlib.h>
#include "ReceiverThread.h"

ReceiverThread::ReceiverThread(const unsigned short portNo)
 : ThreadBase()
 , m_TcpServer(NULL)
{
    m_TcpServer = new TcpServer(portNo);
}

ReceiverThread::~ReceiverThread()
{
    finalize();
}

ResultEnum ReceiverThread::initializeCore()
{
    /* 継承しない場合は何もせず正常を返す */
    return ResultEnum::NormalEnd;
}

void ReceiverThread::doReconnectInitialize(const bool isFirst)
{
    /* 継承しない場合は何もしない */
}

ResultEnum ReceiverThread::doConnectedProc()
{
    /* 継承しない場合は何もせず正常を返す */
    return ResultEnum::NormalEnd;
}

ResultEnum ReceiverThread::finalizeCore()
{
    /* 継承しない場合は何もせず正常を返す */
    return ResultEnum::NormalEnd;
}

ResultEnum ReceiverThread::initialize()
{
    ResultEnum  retVal = ResultEnum::AbnormalEnd;
    
    if (m_TcpServer == NULL)
    {
        m_Logger->LOG_ERROR("[initialize] m_TcpClient allocation failed.\n");
        goto FINISH;
    }

    if (m_TcpServer->Open() != ResultEnum::NormalEnd)
    {
        char logStr[80] = { 0 };
        snprintf(&logStr[0], sizeof(logStr), "[initialize] TcpClient Open failed. errno[%d]\n", m_TcpServer->GetLastError());
        m_Logger->LOG_ERROR(logStr);
        goto FINISH;
    }

    retVal = initializeCore();


FINISH :
    return retVal;
}

ResultEnum ReceiverThread::doProcedure()
{
    ResultEnum retVal = ResultEnum::AbnormalEnd;
    ResultEnum result = ResultEnum::AbnormalEnd;
    bool isFirst = true;
    bool receivable = false;
    EventInfo ev = { 0 };


RECONNECT :

    /* 接続時の初期化処理 */
    doReconnectInitialize(isFirst);
    isFirst = false;

    /* いったん切断する */
    m_TcpServer->Disconnection();

    /* 接続 */
    result = m_TcpServer->Connection();
    if (result != ResultEnum::NormalEnd)
    {
        char logStr[80] = { 0 };
        snprintf(&logStr[0], sizeof(logStr), "[doProcedure] Connection failed. errno[%d]\n", m_TcpServer->GetLastError());
        m_Logger->LOG_ERROR(logStr);

        if (result == ResultEnum::Reconnect)
        {
            goto RECONNECT;
        }
        else
        {
            goto FINISH;
        }
    }

    /* 接続確立時処理 */
    result = doConnectedProc();
    if (result != ResultEnum::NormalEnd)
    {
        char logStr[80] = { 0 };
        snprintf(&logStr[0], sizeof(logStr), "[doProcedure] doConnectedProc failed.\n");
        m_Logger->LOG_ERROR(logStr);

        if (result == ResultEnum::Reconnect)
        {
            goto RECONNECT;
        }
        else
        {
            goto FINISH;
        }
    }

    m_Logger->LOG_INFO("[doProcedure] Main loop enter.\n");
    while (1)
    {
        result = m_TcpServer->IsReceivable(receivable);
        if (result != ResultEnum::NormalEnd)
        {
            char logStr[80] = { 0 };
            snprintf(&logStr[0], sizeof(logStr), "[doProcedure] IsReceivable failed. errno[%d]\n", m_TcpServer->GetLastError());
            m_Logger->LOG_ERROR(logStr);

            if (result == ResultEnum::Reconnect)
            {
                goto RECONNECT;
            }
            else
            {
                goto FINISH;
            }
        }

        if (receivable == false)
        {
            if (isStopRequest() == true)
            {
                m_Logger->LOG_INFO("[doProcedure] Thread stop request.\n");
                break;
            }

            delay(100);
            continue;
        }

        result = m_TcpServer->Receive(&ev, sizeof(EventInfo));
        if (result != ResultEnum::NormalEnd)
        {
            char logStr[80] = { 0 };
            snprintf(&logStr[0], sizeof(logStr), "[doProcedure] Receive failed. errno[%d]\n", m_TcpServer->GetLastError());
            m_Logger->LOG_ERROR(logStr);

            if (result == ResultEnum::Reconnect)
            {
                goto RECONNECT;
            }
            else
            {
                goto FINISH;
            }
        }

        /* 受信データの解析 */
        result = analyze(&ev);
        if (result != ResultEnum::NormalEnd)
        {
            char logStr[80] = { 0 };
            snprintf(&logStr[0], sizeof(logStr), "[doProcedure] analyze failed.\n");
            m_Logger->LOG_ERROR(logStr);

            if (result == ResultEnum::Reconnect)
            {
                goto RECONNECT;
            }
            else
            {
                goto FINISH;
            }
        }
    }
    m_Logger->LOG_INFO("[doProcedure] Main loop exit.\n");

    retVal = ResultEnum::NormalEnd;

FINISH :
    return retVal;
}

ResultEnum ReceiverThread::finalize()
{
    ResultEnum retVal = ResultEnum::AbnormalEnd;

    if (m_TcpServer != NULL)
    {
        m_TcpServer->Close();
        delete m_TcpServer;
        m_TcpServer = NULL;
    }

    retVal = finalizeCore();

    return retVal;
}
