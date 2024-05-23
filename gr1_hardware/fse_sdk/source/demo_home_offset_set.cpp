/**
 * @file demo_home_offset_set.cpp
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
    char *comm_config_set = "{\"method\":\"SET\", \
        \"reqTarget\":\"/home_offset\", \
        \"home_offset\":4993.615724}";
    char ser_msg[1024] = {0};
    fse->demo_broadcase_filter(ABSCODER);
    if (fse->server_ip_filter_num == 0)
    {
        Logger::get_instance()->print_trace_error("Cannot find server\n");
        return 0;
    }

    for (int i = 0; i < fse->server_ip_filter_num; i++)
    {
        std::printf("IP: %s sendto get home offset fse ---> ", fse->server_ip_filter[i].c_str());

        memset(ser_msg, 0, sizeof(ser_msg));
        fse->demo_home_offset_get(fse->server_ip_filter[i], NULL, ser_msg);
        std::printf("%s\n", ser_msg);
        usleep(10);

        memset(ser_msg, 0, sizeof(ser_msg));
        fse->demo_home_offset_set(fse->server_ip_filter[i], comm_config_set, ser_msg);
        std::printf("%s\n", ser_msg);
        sleep(1);

        rapidjson::Document msg_json;
        if (msg_json.Parse(ser_msg).HasParseError())
        {
            Logger::get_instance()->print_trace_error("fi_decode() failed\n");
            return 0;
        }
        std::cout << "status: " << msg_json["status"].GetString() << std::endl;
        sleep(1);

        memset(ser_msg, 0, sizeof(ser_msg));
        fse->demo_home_offset_get(fse->server_ip_filter[i], NULL, ser_msg);
        std::printf("%s\n", ser_msg);
        if (msg_json.Parse(ser_msg).HasParseError())
        {
            Logger::get_instance()->print_trace_error("fi_decode() failed\n");
            return 0;
        }
        // std::cout << "home_offset: " << msg_json["home_offset"].GetFloat() << std::endl;
        std::printf("home_offset: %f \n", msg_json["home_offset"].GetFloat());

        fse->demo_reboot_fse(fse->server_ip_filter[i], NULL, ser_msg);
        std::printf("%s\n", ser_msg);
    }

    return 0;
}
