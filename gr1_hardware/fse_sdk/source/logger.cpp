/**
 ******************************************************************************
 * @file    Project/Source/fi_log.cpp
 * @author  Jason (Chen Xin)
 * @version V 1.0.0
 * @date    20-July-2018
 * @brief
 ******************************************************************************
 */

//-------------------------------- Includes ----------------------------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <string>

#include "function_result.h"
#include "os.h"
#include "logger_state.h"
#include "logger_level.h"
#include "logger.h"

using namespace std;
using namespace Predefine;
using namespace Utils;

enum LoggerState Logger::state_ = LoggerState::STATE_ON;
enum LoggerLevel Logger::level_ = LoggerLevel::LEVEL_DEBUG;

//-------------------------------- Includes ----------------------------------------------------------------------------------------------

//-------------------------------- Macro Definition --------------------------------------------------------------------------------------

//-------------------------------- Macro Definition --------------------------------------------------------------------------------------

//-------------------------------- Variables Definition ----------------------------------------------------------------------------------
// Extern

// Global

// Static

//-------------------------------- Variables Definition ----------------------------------------------------------------------------------

//-------------------------------- Function Definition -----------------------------------------------------------------------------------

/**
 ******************************************************************************
 *
 *	@brief
 *
 ******************************************************************************
 */
Logger::Logger()
{
    this->state_ = STATE_ON;
}

/**
 ******************************************************************************
 *
 *	@brief
 *
 ******************************************************************************
 */
Logger::~Logger()
{
}

/**
 ******************************************************************************
 *
 *	@brief
 *
 ******************************************************************************
 */
int Logger::print(const char *format /*format*/, ...)
{
    if (this->state_ == STATE_ON)
    {
        char buffer[1024];

        va_list args;
        va_start(args, format);
        vsnprintf(buffer, 1024, format, args);

        printf("%s", buffer);
        va_end(args);
    }

result_success:
    return FunctionResult::SUCCESS;

result_failure:
    return FunctionResult::FAILURE;
}

/**
 ******************************************************************************
 *
 *	@brief
 *
 ******************************************************************************
 */
int Logger::print_line(const char *format /*format*/, ...)
{
    if (this->state_ == STATE_ON)
    {
        char buffer[1024];

        va_list args;
        va_start(args, format);
        vsnprintf(buffer, 1024, format, args);

        printf("%s", buffer);
        va_end(args);
    }

result_success:
    return FunctionResult::SUCCESS;

result_failure:
    return FunctionResult::FAILURE;
}

/**
 ******************************************************************************
 *
 *	@brief Print log with time-stamp [TIME: xxx]
 *
 ******************************************************************************
 */
int Logger::print_trace(const char *format /*format*/, ...)
{
    if (this->state_ == STATE_ON)
    {
        char buffer[1024];
        double time_in_s = 0;

        OS::get_instance()->get_time_in_s(&time_in_s);

        printf("[TIME: %f] ", time_in_s);

        va_list args;
        va_start(args, format);
        vsnprintf(buffer, 1024, format, args);

        printf("%s", buffer);
        va_end(args);
    }

result_success:
    return FunctionResult::SUCCESS;

result_failure:
    return FunctionResult::FAILURE;
}

/**
 ******************************************************************************
 *
 *	@brief Print log with time-stamp [TIME: xxx]
 *
 ******************************************************************************
 */
int Logger::print_trace(enum LoggerLevel level, const char *format /*format*/, ...)
{
    if (this->state_ == STATE_ON)
    {
        char buffer[1024];
        double time_in_s = 0;

        OS::get_instance()->get_time_in_s(&time_in_s);

        printf("[TIME: %f] ", time_in_s);

        va_list args;
        va_start(args, format);
        vsnprintf(buffer, 1024, format, args);

        switch (level)
        {
        case LoggerLevel::LEVEL_DEBUG:
            printf("\033[32m");
            printf("%s", buffer);
            printf("\033[0m");
            break;

        case LoggerLevel::LEVEL_WARNING:
            printf("\033[33m");
            printf("%s", buffer);
            printf("\033[0m");
            break;

        case LoggerLevel::LEVEL_ERROR:
            printf("\033[31m");
            printf("%s", buffer);
            printf("\033[0m");
            break;

        default:
            printf("%s", buffer);
            break;
        }

        va_end(args);
    }

result_success:
    return FunctionResult::SUCCESS;

result_failure:
    return FunctionResult::FAILURE;
}

/**
 ******************************************************************************
 *
 *	@brief Print log with time-stamp [TIME: xxx]
 *
 ******************************************************************************
 */
int Logger::print_trace_debug(const char *format /*format*/, ...)
{
    if (this->state_ == STATE_ON)
    {
        char buffer[1024];
        double time_in_s = 0;

        OS::get_instance()->get_time_in_s(&time_in_s);

        printf("[TIME: %f] ", time_in_s);

        va_list args;
        va_start(args, format);
        vsnprintf(buffer, 1024, format, args);

        printf("\033[32m");
        printf("%s", buffer);
        va_end(args);
        printf("\033[0m");
    }

result_success:
    return FunctionResult::SUCCESS;

result_failure:
    return FunctionResult::FAILURE;
}

/**
 ******************************************************************************
 *
 *	@brief Print log with time-stamp [TIME: xxx]
 *
 ******************************************************************************
 */
int Logger::print_trace_warning(const char *format /*format*/, ...)
{
    if (this->state_ == STATE_ON)
    {
        char buffer[1024];
        double time_in_s = 0;

        OS::get_instance()->get_time_in_s(&time_in_s);

        printf("[TIME: %f] ", time_in_s);

        va_list args;
        va_start(args, format);
        vsnprintf(buffer, 1024, format, args);

        printf("\033[33m");
        printf("%s", buffer);
        va_end(args);
        printf("\033[0m");
    }

result_success:
    return FunctionResult::SUCCESS;

result_failure:
    return FunctionResult::FAILURE;
}

/**
 ******************************************************************************
 *
 *	@brief Print log with time-stamp [TIME: xxx]
 *
 ******************************************************************************
 */
int Logger::print_trace_error(const char *format /*format*/, ...)
{
    if (this->state_ == STATE_ON)
    {
        char buffer[1024];
        double time_in_s = 0;

        OS::get_instance()->get_time_in_s(&time_in_s);

        printf("[TIME: %f] ", time_in_s);

        va_list args;
        va_start(args, format);
        vsnprintf(buffer, 1024, format, args);

        printf("\033[31m");
        printf("%s", buffer);
        va_end(args);
        printf("\033[0m");
    }

result_success:
    return FunctionResult::SUCCESS;

result_failure:
    return FunctionResult::FAILURE;
}

/**
 ******************************************************************************
 *
 *	@brief Print log information to file
 *
 ******************************************************************************
 */
int Logger::print_file(const char *format /*format*/, ...)
{
    if (this->state_ == STATE_ON)
    {
        FILE *file_id;
        file_id = fopen("log.txt", "a+");

        if (file_id == NULL)
        {
            goto result_failure;
        }

        char buffer[1024];

        va_list args;
        va_start(args, format);
        vsnprintf(buffer, 1024, format, args);

        fprintf(file_id, "%s", buffer);
        va_end(args);

        fclose(file_id);
    }

result_success:
    return FunctionResult::SUCCESS;

result_failure:
    return FunctionResult::FAILURE;
}

/**
 ******************************************************************************
 *
 *	@brief Print log information to file
 *
 ******************************************************************************
 */
int Logger::print_file_trace(const char *format /*format*/, ...)
{
    if (this->state_ == STATE_ON)
    {
        FILE *file_id;
        file_id = fopen("log.txt", "a+");

        if (file_id == NULL)
        {
            goto result_failure;
        }

        char buffer[1024];
        double time_in_s = 0;

        OS::get_instance()->get_time_in_s(&time_in_s);

        fprintf(file_id, "[TIME: %f] ", time_in_s);

        va_list args;
        va_start(args, format);
        vsnprintf(buffer, 1024, format, args);

        fprintf(file_id, "%s", buffer);
        va_end(args);

        fclose(file_id);
    }

result_success:
    return FunctionResult::SUCCESS;

result_failure:
    return FunctionResult::FAILURE;
}

int Logger::add_to_log_buffer(string log_buffer)
{
    this->log_buffer.push_back(log_buffer);
    return this->log_buffer.size();
}

int Logger::clear_log_buffer()
{
    if (this->log_buffer.size() > 0)
    {
        this->log_buffer.clear();
    }
    return this->log_buffer.size();
}

int Logger::print_log_buffer()
{
    if (this->log_buffer.size() > 0)
    {
        for (int i = 0; i < this->log_buffer.size(); i++)
        {
            printf("%s", this->log_buffer[i].c_str());
        }
    }
    return this->log_buffer.size();
}

int Logger::print_log_buffer_to_file(string file_name)
{
    if (this->log_buffer.size() > 0)
    {
        FILE *file_id;
        file_id = fopen(file_name.c_str(), "a+");

        if (file_id == NULL)
        {
            return FunctionResult::FAILURE;
        }

        for (int i = 0; i < this->log_buffer.size(); i++)
        {
            fprintf(file_id, "%s", this->log_buffer[i].c_str());
        }

        fclose(file_id);
    }
    return this->log_buffer.size();
}

//-------------------------------- Function Definition -----------------------------------------------------------------------------------

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
