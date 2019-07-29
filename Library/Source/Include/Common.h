#pragma once

#include <wiringPi.h>

/* ���ʒ萔 */
typedef enum
{
    NormalEnd,          /* ����I�� */
    AbnormalEnd,        /* �ُ�I�� */
    Reconnect,          /* �Đڑ� (�ʐM�p) */
} ResultEnum;

/* �C�x���g�\���� */
typedef struct
{
    long From;          /* ���M�� */
    long To;            /* ���� */
    long Code;          /* �C�x���g�R�[�h */
    ResultEnum Result;  /* ���s���� */
    long lParam[5];     /* long �^�p�����[�^ */
    long fParam[10];    /* float �^�p�����[�^ */
} EventInfo;

#define LOG_EVCODE_OUTPUT       (1)     /* ���O�o�� */
#define LOG_EVCODE_STOP         (2)     /* ���O��~ */
#define LOG_EVCODE_RESTART      (3)     /* ���O�ĊJ */
