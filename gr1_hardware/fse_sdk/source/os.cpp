/**
 ******************************************************************************
 * @file    Project/Application/OS/fi_os.c
 * @author  Jason (Chen Xin)
 * @version V 1.0.0
 * @date    20-July-2018
 * @brief
 ******************************************************************************
 */

//-------------------------------- Includes ----------------------------------------------------------------------------------------------

#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>

#include "function_result.h"
#include "os.h"

using namespace Predefine;
using namespace Utils;

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
 ****************************************************************************
 *
 *	@brief : INIT
 *
 *****************************************************************************
 */
OS::OS()
{
    this->get_time_in_s(&this->start_time_in_s_);
}

/**
 ****************************************************************************
 *
 *	@brief : DEINIT
 *
 *****************************************************************************
 */
OS::~OS()
{
}

// OS *OS::get_instance()
// {
//     static OS instance;
//     return &instance;
// }

/**
 ****************************************************************************
 *
 *	@brief : GET Time in us
 *
 *	@details :
 *
 *****************************************************************************
 */
int OS::get_time_in_us(double *us)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    *us = (double)tv.tv_sec * 1000000 + tv.tv_usec - this->start_time_in_s_ * 1000000;
    return FunctionResult::SUCCESS;
}

/**
 ****************************************************************************
 *
 *	@brief : GET Time in ms
 *
 *	@details :
 *
 *****************************************************************************
 */
int OS::get_time_in_ms(double *ms)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    *ms = (double)tv.tv_sec * 1000 + tv.tv_usec * 0.001f - this->start_time_in_s_ * 1000;
    return FunctionResult::SUCCESS;
}

/**
 ****************************************************************************
 *
 *	@brief : GET Time in s
 *
 *	@details :
 *
 *****************************************************************************
 */
int OS::get_time_in_s(double *s)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    *s = (double)tv.tv_sec + tv.tv_usec * 0.000001f - this->start_time_in_s_;
    return FunctionResult::SUCCESS;
}

/**
 ****************************************************************************
 *
 *	@brief : GET Time in tick
 *
 *	@details :
 *
 *****************************************************************************
 */
int OS::get_time_in_tick(double *tick)
{
    *tick = this->tick_;
    return FunctionResult::SUCCESS;
}

/**
 ******************************************************************************
 *
 *	@brief SLEEP s
 *
 *	@details
 *
 ******************************************************************************
 */
int OS::sleep_s(float s)
{
    return FunctionResult::SUCCESS;
}

/**
 ******************************************************************************
 *
 *	@brief SLEEP ms
 *
 *	@details
 *
 ******************************************************************************
 */
int OS::sleep_ms(float ms)
{
    double start_time = 0;
    double current_time = 0;

    this->get_time_in_ms(&start_time);
    this->get_time_in_ms(&current_time);

    while (1)
    {
        if (current_time - ms > start_time)
        {
            break;
        }

        this->get_time_in_ms(&current_time);
    }

    return FunctionResult::SUCCESS;
}

/**
 ******************************************************************************
 *
 *	@brief SLEEP us
 *
 *	@details
 *
 ******************************************************************************
 */
int OS::sleep_us(float us)
{
    double start_time = 0;
    double current_time = 0;

    this->get_time_in_us(&start_time);
    this->get_time_in_us(&current_time);

    while (1)
    {
        if (current_time - us > start_time)
        {
            break;
        }

        this->get_time_in_us(&current_time);
    }

    return FunctionResult::SUCCESS;
}

/**
 ******************************************************************************
 *
 *	@brief SLEEP tick
 *
 *	@details
 *
 ******************************************************************************
 */
int OS::sleep_tick(float tick)
{
    for (int i = 0; i < tick; i++)
    {
    }

    return FunctionResult::SUCCESS;
}

/**
 ******************************************************************************
 *
 *	@brief DELAY s
 *
 *	@details
 *
 ******************************************************************************
 */
int OS::delay_s(float s)
{
    usleep(s * 1000 * 1000);
    return FunctionResult::SUCCESS;
}

/**
 ******************************************************************************
 *
 *	@brief DELAY ms
 *
 *	@details
 *
 ******************************************************************************
 */
int OS::delay_ms(float ms)
{
    usleep(ms * 1000);
    return FunctionResult::SUCCESS;
}

/**
 ******************************************************************************
 *
 *	@brief DELAY us
 *
 *	@details
 *
 ******************************************************************************
 */
int OS::delay_us(float us)
{
    usleep(us);
    return FunctionResult::SUCCESS;
}

/**
 ******************************************************************************
 *
 *	@brief DELAY tick
 *
 *	@details
 *
 ******************************************************************************
 */
int OS::delay_tick(float tick)
{
    usleep(tick);
    return FunctionResult::SUCCESS;
}

/**
 * @brief return interval time unit:s
 */
double OS::get_interval_time_in_s(struct timeval ts, struct timeval te)
{
    return (te.tv_sec - ts.tv_sec) + (te.tv_usec - ts.tv_usec) / 1000000.0;
}

//-------------------------------- Function Definition -----------------------------------------------------------------------------------

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
