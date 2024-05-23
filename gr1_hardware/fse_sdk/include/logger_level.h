#ifndef _LOGGER_LEVEL_H_
#define _LOGGER_LEVEL_H_

namespace Utils
{
    enum LoggerLevel
    {
        LEVEL_INFO = 0x00,
        LEVEL_DEBUG = 0x01,
        LEVEL_WARNING = 0x02,
        LEVEL_ERROR = 0x03,
    };
}

#endif // _LOGGER_LEVEL_H_