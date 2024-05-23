/**
 * @file demo_ctrl_config_save.cpp
 * @author Afer
 * @brief
 * @version 0.1
 * @date 2023-12-21
 * @note 1.recvfrom failed, because server havenot send msg
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
    char ser_msg[1024] = {0};
    fse->demo_broadcase_filter(ABSCODER);
    if (fse->server_ip_filter_num == 0)
    {
        Logger::get_instance()->print_trace_error("Cannot find server\n");
        return 0;
    }

    for (int i = 0; i < fse->server_ip_filter_num; i++)
    {
        std::printf("IP: %s sendto save ctrl_config ---> ", fse->server_ip_filter[i].c_str());
        fse->demo_ctrl_config_save(fse->server_ip_filter[i], NULL, ser_msg);
        // std::printf("%s\n", ser_msg);

        // rapidjson::Document msg_json;
        // if (msg_json.Parse(ser_msg).HasParseError())
        // {
        //     logger->fi_logger_error("fi_decode() failed");
        //     return 0;
        // }
        // std::cout << "status: " << msg_json["status"].GetString() << std::endl;
    }
    return 0;
}
