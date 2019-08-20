#pragma once

#include "Socket/TcpClient/TcpClient.h"
#include "Queue/Queue.h"
#include "ThreadBase/ThreadBase.h"

/* ���}�C�R���ւ̑��M���s���X���b�h�̊��ƂȂ�N���X */
/* �����̃g���K�͌p����Ŏ��R�Ɏw��ł��� (createSendData ���\�b�h�� true ��Ԃ��Ƒ��M�����) */
/* �L���[�Ɏ�M�������e�𑼂̃}�C�R���ɓ]������ */
class SenderThread : public ThreadBase
{
public :

    SenderThread(char* const ipAddress, const unsigned short portNo);
    virtual ~SenderThread();

protected :

    virtual ResultEnum initializeCore();
    virtual void doReconnectInitialize(const bool isFirst);
    virtual ResultEnum doConnectedProc();
    virtual bool createSendData(EventInfo* const ev) = 0;
    virtual ResultEnum finalizeCore();

private :

    TcpClient* m_TcpClient;

    ResultEnum initialize();
    ResultEnum doProcedure();
    ResultEnum finalize();

};
