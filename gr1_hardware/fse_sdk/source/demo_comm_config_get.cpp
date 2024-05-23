/**
 * @file demo_comm_config_get.cpp
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
    rapidjson::Document msg_json;

    char ser_msg[1024] = {0};
    fse->demo_broadcase_filter(ABSCODER);
    if (fse->server_ip_filter_num == 0)
    {
        Logger::get_instance()->print_trace_error("Cannot find server\n");
        return 0;
    }
    std::string ser_list[254] = {""};
    memset(ser_list, 0, sizeof(ser_list));
    memcpy(ser_list, fse->server_ip_filter, sizeof(ser_list));
    int ser_num = fse->server_ip_filter_num;
    while(1)
    {
        for (int i = 0; i < ser_num; i++)
        {
            std::printf("IP: %s sendto get_comm_config ---> ", ser_list[i].c_str());
            fse->demo_comm_config_get(ser_list[i], NULL, ser_msg);
            std::printf("%s\n", ser_msg);

            if (msg_json.Parse(ser_msg).HasParseError())
            {
                Logger::get_instance()->print_trace_error("fi_decode() failed\n");
                sleep(100);
                return 0;
            }
            std::cout << "status: " << msg_json["status"].GetString() << std::endl;
        }
        usleep(500);
    }

    return 0;
}
