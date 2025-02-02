#pragma once

#include <wiringPi.h>

/* 結果定数 */
typedef enum
{
    NormalEnd,          /* 正常終了 */
    AbnormalEnd,        /* 異常終了 */
    Reconnect,          /* 再接続 (通信用) */
} ResultEnum;

/* イベント構造体 */
typedef struct
{
    long Code;          /* イベントコード */
    ResultEnum Result;  /* 実行結果 */
    long lParam[5];     /* long 型パラメータ */
    long fParam[10];    /* float 型パラメータ */
} EventInfo;

/* ログ出力プロセス 関連定義 */
#define LOG_WRITER_QUEUE_NAME   "LogWriter"

#define LOG_EVCODE_OUTPUT       (1)     /* ログ出力 */
#define LOG_EVCODE_STOP         (2)     /* ログ停止 */
#define LOG_EVCODE_RESTART      (3)     /* ログ再開 */
#define LOG_EVCODE_FINISH       (99)    /* ログ終了 */

/* IP アドレス定義 (有線) */
#define COMMANDER_IP_ADDRESS        "192.168.3.1"   /* 司令塔マイコン IP アドレス */
#define AROUND_CAMERA_IP_ADDRESS    "192.168.3.2"   /* 360°カメラマイコン IP アドレス */
#define FRONT_CAMERA_IP_ADDRESS     "192.168.3.3"   /* 前方カメラマイコン IP アドレス */

/* ポート番号定義 (有線) */
#define COMMANDER_TO_AC_PORT        (10001)         /* 司令塔マイコン → 360°カメラマイコン 通信ポート番号 */
#define COMMANDER_TO_FC_PORT        (10001)         /* 司令塔マイコン → 前方カメラマイコン 通信ポート番号 */
#define MOTOR_TO_COMMANDER_PORT     (10000)         /* モータマイコン → 司令塔マイコン 通信スレッド ポート番号 */
#define AC_TO_COMMANDER_PORT        (10001)         /* 360°カメラマイコン → 司令塔マイコン 状態通信スレッド ポート番号 */
#define FC_TO_COMMANDER_PORT        (10002)         /* 前方カメラマイコン → 司令塔マイコン 状態通信スレッド ポート番号 */

/* IP アドレス定義 (Wifi) */
#define COMMANDER_WIFI_IP           "10.0.0.1"      /* 司令塔マイコン Wifi IP アドレス */
#define ACCESS_STATION_1_WIFI_IP    "10.0.0.11"     /* アクセスステーション1 Wifi IP アドレス */
#define ACCESS_STATION_2_WIFI_IP    "10.0.0.12"     /* アクセスステーション2 Wifi IP アドレス */
#define ACCESS_STATION_3_WIFI_IP    "10.0.0.13"     /* アクセスステーション3 Wifi IP アドレス */
#define REMOCON_WIFI_IP             "10.0.0.101"    /* リモコンマイコン Wifi IP アドレス */
#define TOOL_WIFI_IP                "10.0.0.102"    /* 設定ツール PC Wifi IP アドレス */

/* ポート番号定義 (Wifi) */
#define REMOCON_TO_COMMANDER_PORT   (20000)         /* リモコンマイコン → 司令塔マイコン 通信スレッド ポート番号 */
#define TOOL_TO_COMMANDER_PORT      (20001)         /* 設定ツール → 司令塔マイコン 通信スレッド ポート番号 */
