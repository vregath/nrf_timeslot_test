
#include <SEGGER_RTT.h>
#include <stdarg.h>

#define LOG_BUFFER_SIZE	512
static char logBuffer[LOG_BUFFER_SIZE];

void logPrint(const char *format, ...) {

    int count = 0;
	
    va_list va;
    va_start(va, format);
    count += vsnprintf(logBuffer + count, LOG_BUFFER_SIZE - count, format, va);
    va_end(va);
	
    count += snprintf(logBuffer + count, LOG_BUFFER_SIZE - count, "\n");
    logBuffer[count] = 0;

    //va_list argptr;
    //va_start(argptr,format);
    SEGGER_RTT_printf(0, logBuffer);
    //va_end(argptr);
    
}
