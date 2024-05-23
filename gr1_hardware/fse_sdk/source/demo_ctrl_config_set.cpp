#include "main.h"


using namespace Sensor;
using namespace Utils;
using namespace Predefine;
FSE *fse = new FSE();
int main()
{
    char *comm_config_set = "{\"method\":\"SET\", \
        \"reqTarget\":\"/config\", \
        \"property\":\"\", \
        \"home_offset\":5}";

    char ser_msg[1024] = {0};
    fse->demo_broadcase_filter(ABSCODER);
    if (fse->server_ip_filter_num == 0)
    {
        Logger::get_instance()->print_trace_error("Cannot find server\n");
        return 0;
    }

    for (int i = 0; i < fse->server_ip_filter_num; i++)
    {
        std::printf("IP: %s sendto reboot fse ---> ", fse->server_ip_filter[i].c_str());

        memset(ser_msg, 0, sizeof(ser_msg));
        fse->demo_ctrl_config_get(fse->server_ip_filter[i], NULL, ser_msg);
        std::printf("%s\n", ser_msg);
        usleep(10);

        memset(ser_msg, 0, sizeof(ser_msg));
        fse->demo_ctrl_config_set(fse->server_ip_filter[i], comm_config_set, ser_msg);
        std::printf("%s\n", ser_msg);
        sleep(3);

        memset(ser_msg, 0, sizeof(ser_msg));
        fse->demo_ctrl_config_get(fse->server_ip_filter[i], NULL, ser_msg);
        std::printf("%s\n", ser_msg);
        sleep(1);

        memset(ser_msg, 0, sizeof(ser_msg));
        fse->demo_reboot_fse(fse->server_ip_filter[i], NULL, ser_msg);
        Logger::get_instance()->print_trace_debug("%s\n", ser_msg);

        rapidjson::Document msg_json;
        if (msg_json.Parse(ser_msg).HasParseError())
        {
            Logger::get_instance()->print_trace_error("fi_decode() failed\n");
            return 0;
        }
        std::cout << "status: " << msg_json["status"].GetString() << std::endl;
    }

    return 0;
}
