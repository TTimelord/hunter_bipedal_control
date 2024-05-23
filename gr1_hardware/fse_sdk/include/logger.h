#ifndef _LOGGER_H_
#define _LOGGER_H_

#include <string>
#include <vector>

#include "logger_state.h"
#include "logger_level.h"

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_RESET "\x1b[0m"

using namespace std;

namespace Utils
{
    class Logger
    {
    public:
        static enum LoggerState state_;
        static enum LoggerLevel level_;

    private:
        Logger();
        ~Logger();

    public:
        static Logger *get_instance()
        {
            static Logger instance;
            return &instance;
        }

    public:
        int print(const char *format /*format*/, ...);
        int print_line(const char *format /*format*/, ...);
        int print_trace(const char *format /*format*/, ...);
        int print_trace(enum LoggerLevel level, const char *format /*format*/, ...);
        int print_trace_debug(const char *format /*format*/, ...);
        int print_trace_warning(const char *format /*format*/, ...);
        int print_trace_error(const char *format /*format*/, ...);
        int print_file(const char *format /*format*/, ...);
        int print_file_trace(const char *format /*format*/, ...);

    public:
        vector<string> log_buffer;

        int add_to_log_buffer(string log_);
        int clear_log_buffer();
        int print_log_buffer();
        int print_log_buffer_to_file(string file_name);
    };
} // namespace FI

#endif
