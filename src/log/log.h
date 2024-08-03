
#if 0  // debug log config
#include <uart/uart.h>
#define  DBGLOGI    LOGI
#define  DBGLOGW    LOGW
#define  DBGLOGE    LOGE
#define  DBGWrite   uartWrite
#elif 1 // RTT debug log
#include <SEGGER_RTT.h>
#define  DBGLOGI(tag, format, ...)    SEGGER_RTT_printf(0, "[I]%s:" format "\n", tag, ##__VA_ARGS__)
#define  DBGLOGW(tag, format, ...)    SEGGER_RTT_printf(0, "[W]%s:" format "\n", tag, ##__VA_ARGS__)
#define  DBGLOGE(tag, format, ...)    SEGGER_RTT_printf(0, "[E]%s:" format "\n", tag, ##__VA_ARGS__)
#define  DBGWrite(...)   SEGGER_RTT_WriteString(0, __VA_ARGS__)
//#define  DBGWrite(args...)   SEGGER_RTT_WriteString(0, args) // GNU
#else // NO debug log
#define  DBG_NONE(...)
#define  DBGLOGI    DBG_NONE
#define  DBGLOGW    DBG_NONE
#define  DBGLOGE    DBG_NONE
#define  DBGWrite   DBG_NONE
#endif
