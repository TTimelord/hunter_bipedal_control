/*******************************************************************************
 * @file    Project/Include/OS/os.h
 * @author  Jason (Chen Xin)
 * @version V 1.0.0
 * @date    01-09-2019
 * @brief
 ******************************************************************************/

#ifndef _OS_H_
#define _OS_H_

//-------------------------------- Include Definition --------------------------------------------------------------------------------------

//-------------------------------- Include Definition --------------------------------------------------------------------------------------

//-------------------------------- Macro Definition --------------------------------------------------------------------------------------

//-------------------------------- Macro Definition --------------------------------------------------------------------------------------

//-------------------------------- Variables Definition ----------------------------------------------------------------------------------
// Extern

// Global

// Static

//-------------------------------- Variables Definition ----------------------------------------------------------------------------------

//-------------------------------- Model Definition --------------------------------------------------------------------------------------

namespace Utils
{
    class OS
    {
    public:
        double start_time_in_s_;
        double us_;
        double ms_;
        double s_;
        double tick_;

    private:
        OS();
        ~OS();

    public:
        static OS *get_instance()
        {
            static OS instance;
            return &instance;
        }

    public:
        int get_time_in_us(double *us);
        int get_time_in_ms(double *ms);
        int get_time_in_s(double *s);
        int get_time_in_tick(double *tick);

    public:
        // make current thread to sleep, will not give out execution authority
        int sleep_s(float s);
        int sleep_ms(float ms);
        int sleep_us(float us);
        int sleep_tick(float tick);

        // make current thread to sleep, will give out execution authority
        int delay_s(float s);
        int delay_ms(float ms);
        int delay_us(float us);
        int delay_tick(float tick);

        double get_interval_time_in_s(struct timeval ts, struct timeval te);
    };
} // namespace FI

//-------------------------------- Model Definition --------------------------------------------------------------------------------------

//-------------------------------- Function Definition --------------------------------------------------------------------------------------

//-------------------------------- Function Definition --------------------------------------------------------------------------------------

#endif /* _FI_OS_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
