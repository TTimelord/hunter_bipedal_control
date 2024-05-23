/**
 * @file demo_lookup_AbsEncoder.cpp
 * @author Afer
 * @brief
 * @version 0.1
 * @date 2023-12-21
 * @note pass-test
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "main.h"
using namespace Sensor;
using namespace Utils;
using namespace Predefine;
FSE *fse = new FSE();
int main()
{
    fse->demo_broadcase_filter("AbsEncoder");
    for (int i = 0; i < fse->server_ip_filter_num; i++)
    {
        std::cout << fse->server_ip_filter[i] << std::endl;
    }

    return 0;
}
