/**
 * @file fse_list.cpp
 * @author Afer
 * @brief
 * @version 0.1
 * @date 2023-12-21
 * @note
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <fstream>
#include <sstream>
#include <vector>
#include "main.h"

using namespace Sensor;
using namespace Utils;
using namespace Predefine;

FSE *fse = new FSE();

std::string int_array_to_ip(const std::vector<int> &int_array)
{
    std::ostringstream oss;
    for (size_t i = 0; i < int_array.size(); ++i)
    {
        if (i > 0)
        {
            oss << ".";
        }
        oss << int_array[i];
    }
    return oss.str();
}

int main()
{
    // read json ../config/config.json
    std::ifstream json_file("../config/config.json");
    std::stringstream json_buff;
    json_buff << json_file.rdbuf();
    std::string json_string = json_buff.str();

    // parse json string
    rapidjson::Document msg_json;
    if (msg_json.Parse(json_string.c_str()).HasParseError())
    {
        Logger::get_instance()->print_trace_error("fi_decode() failed\n");
        return FunctionResult::FAILURE;
    }
    // test
    std::cout << "Device: " << msg_json["Device"].GetString() << std::endl;

    // find key was enable
    if ((msg_json["Device"].GetString()) == "FSE")
    {
        Logger::get_instance()->print_trace_error("Please set \"Device\":\"FSE\"\n");
        return FunctionResult::FAILURE;
    }

    const rapidjson::Value &commandType = msg_json["Command"];
    const rapidjson::Value &param = msg_json["Param"];

    // boradcast
    char ser_msg[1024] = {0};
    fse->demo_broadcase_filter(ABSCODER);
    if (fse->server_ip_filter_num == 0)
    {
        Logger::get_instance()->print_trace_error("Cannot find server\n");
        return FunctionResult::FAILURE;
    }
    std::string ser_list[254] = {""};
    memset(ser_list, 0, sizeof(ser_list));
    memcpy(ser_list, fse->server_ip_filter, sizeof(ser_list));
    int ser_num = fse->server_ip_filter_num;

    if (commandType["comm_config_get"].GetBool())
    {
        // enable
        for (int i = 0; i < ser_num; i++)
        {
            memset(ser_msg, 0, sizeof(ser_msg));
            Logger::get_instance()->print_trace_warning("IP: %s sendto get_comm_config ---> \n", ser_list[i].c_str());

            fse->demo_comm_config_get(ser_list[i], NULL, ser_msg);
            Logger::get_instance()->print_trace_debug("recv msg:%s\n", ser_msg);
        }
        usleep(1000); // The necessary delay for running multiple scripts
    }

    if (commandType["comm_config_set"].GetBool())
    {
        for (int i = 0; i < ser_num; i++)
        {
            // step1: get current communicational config
            memset(ser_msg, 0, sizeof(ser_msg));
            Logger::get_instance()->print_trace_warning("IP: %s sendto get_comm_config ---> \n", ser_list[i].c_str());
            fse->demo_comm_config_get(ser_list[i], NULL, ser_msg);
            Logger::get_instance()->print_trace_debug("recv msg:%s\n", ser_msg);
            sleep(1);

            // step2: make json msg
            const rapidjson::Value &comm_set_msg = param["comm_config_set_param"];
            rapidjson::StringBuffer buffer;
            rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);

            writer.StartObject();
            writer.Key("method");
            writer.String("SET");

            writer.Key("reqTarget");
            writer.String("/config");

            writer.Key("property");
            writer.String("");

            writer.Key("name");
            writer.String("FSE");

            writer.Key("DHCP_enable");
            writer.Bool(comm_set_msg["DHCP_enable"].GetBool());

            writer.Key("SSID");
            writer.String(comm_set_msg["SSID"].GetString());

            writer.Key("password");
            writer.String(comm_set_msg["password"].GetString());

            writer.Key("static_IP");
            writer.StartArray();
            writer.Int(comm_set_msg["static_IP"][i][0].GetInt());
            writer.Int(comm_set_msg["static_IP"][i][1].GetInt());
            writer.Int(comm_set_msg["static_IP"][i][2].GetInt());
            writer.Int(comm_set_msg["static_IP"][i][3].GetInt());
            writer.EndArray();

            writer.Key("gateway");
            writer.StartArray();
            writer.Int(192);
            writer.Int(168);
            writer.Int(137);
            writer.Int(1);
            writer.EndArray();

            writer.Key("subnet_mask");
            writer.StartArray();
            writer.Int(255);
            writer.Int(255);
            writer.Int(255);
            writer.Int(0);
            writer.EndArray();

            writer.Key("dns_1");
            writer.StartArray();
            writer.Int(114);
            writer.Int(114);
            writer.Int(114);
            writer.Int(114);
            writer.EndArray();

            writer.Key("dns_2");
            writer.StartArray();
            writer.Int(8);
            writer.Int(8);
            writer.Int(8);
            writer.Int(8);
            writer.EndArray();

            writer.EndObject();

            std::string comm_set_config_msg = buffer.GetString();
            char send_set_config[1024] = {0};
            comm_set_config_msg.copy(send_set_config, sizeof(send_set_config) - 1);
            send_set_config[sizeof(send_set_config) - 1] = '\0';

            // step3: send msg to absEncoder
            memset(ser_msg, 0, sizeof(ser_msg));
            Logger::get_instance()->print_trace_warning("IP: %s sendto set_comm_config ---> \n", ser_list[i].c_str());
            fse->demo_comm_config_set(ser_list[i], send_set_config, ser_msg);
            Logger::get_instance()->print_trace_debug("recv msg:%s\n", ser_msg);
            sleep(1);

            // step4: reboot fse
            memset(ser_msg, 0, sizeof(ser_msg));
            Logger::get_instance()->print_trace_warning("IP: %s sendto reboot fse ---> \n", ser_list[i].c_str());
            fse->demo_reboot_fse(ser_list[i], NULL, ser_msg);
            Logger::get_instance()->print_trace_debug("recv msg:%s\n", ser_msg);
            Logger::get_instance()->print_trace_warning("restarting ...\n");
            for (uint8_t i = 0; i < 7; i++)
            {
                sleep(1);
                Logger::get_instance()->print_trace(".\n");
            }

            // step5: if static ip was changed, it need ip newly.
            std::vector<int> int_array = {comm_set_msg["static_IP"][i][0].GetInt(),
                                          comm_set_msg["static_IP"][i][1].GetInt(), comm_set_msg["static_IP"][i][2].GetInt(), comm_set_msg["static_IP"][i][3].GetInt()};
            std::string new_ip = int_array_to_ip(int_array);

            // step6: if new_ip = broadcast_ip, then broadcast_ip will be used, else, new_ip used
            /**
             * @bug it will debug question that ip was changed, cannot receive message for getting config
             */
            memset(ser_msg, 0, sizeof(ser_msg));
            if (new_ip == ser_list[i])
            {
                memset(ser_msg, 0, sizeof(ser_msg));
                Logger::get_instance()->print_trace_warning("IP: %s sendto get_comm_config ---> \n", ser_list[i].c_str());
                fse->demo_comm_config_get(ser_list[i], NULL, ser_msg);
            }
            else
            {
                memset(ser_msg, 0, sizeof(ser_msg));
                Logger::get_instance()->print_trace_warning("IP: %s sendto get_comm_config ---> \n", new_ip.c_str());
                fse->demo_comm_config_get(new_ip, NULL, ser_msg);
            }
            Logger::get_instance()->print_trace_debug("recv msg:%s\n", ser_msg);
        }
        usleep(1000); // The necessary delay for running multiple scripts
    }

    if (commandType["ctrl_config_get"].GetBool())
    {
        for (int i = 0; i < ser_num; i++)
        {
            memset(ser_msg, 0, sizeof(ser_msg));
            Logger::get_instance()->print_trace_debug("IP: %s sendto get fse ctrl config ---> ", ser_list[i].c_str());
            fse->demo_ctrl_config_get(ser_list[i], NULL, ser_msg);
            Logger::get_instance()->print_trace_debug("%s\n", ser_msg);
        }
        usleep(1000); // The necessary delay for running multiple scripts
    }

    if (commandType["ctrl_config_set"].GetBool())
    {
        for (int i = 0; i < ser_num; i++)
        {
            // step1: getting control config
            memset(ser_msg, 0, sizeof(ser_msg));
            Logger::get_instance()->print_trace_debug("IP: %s sendto get fse ctrl config ---> ", ser_list[i].c_str());
            fse->demo_ctrl_config_get(ser_list[i], NULL, ser_msg);
            Logger::get_instance()->print_trace_debug("%s\n", ser_msg);
            sleep(1);

            // step2: make json
            const rapidjson::Value &ctrl_set_msg = param["ctrl_config_set_param"];
            rapidjson::StringBuffer buffer;
            rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);

            writer.StartObject();
            writer.Key("method");
            writer.String("SET");

            writer.Key("reqTarget");
            writer.String("/config");

            writer.Key("property");
            writer.String("");

            writer.Key("home_offset");
            writer.Int(ctrl_set_msg["home_offset"].GetInt());

            writer.EndObject();

            std::string ctrl_set_config_msg = buffer.GetString();
            char send_set_config[1024] = {0};
            ctrl_set_config_msg.copy(send_set_config, sizeof(send_set_config) - 1);
            send_set_config[sizeof(send_set_config) - 1] = '\0';

            // step3: set control config
            memset(ser_msg, 0, sizeof(ser_msg));
            Logger::get_instance()->print_trace_debug("IP: %s sendto set fse ctrl config ---> ", ser_list[i].c_str());
            fse->demo_ctrl_config_set(ser_list[i], send_set_config, ser_msg);
            Logger::get_instance()->print_trace_debug("%s\n", ser_msg);
            sleep(1);

            // step4: reboot fse
            memset(ser_msg, 0, sizeof(ser_msg));
            Logger::get_instance()->print_trace_warning("IP: %s sendto reboot fse ---> \n", ser_list[i].c_str());
            fse->demo_reboot_fse(ser_list[i], NULL, ser_msg);
            Logger::get_instance()->print_trace_debug("recv msg:%s\n", ser_msg);
            Logger::get_instance()->print_trace_warning("restarting ...\n");
            for (uint8_t i = 0; i < 7; i++)
            {
                sleep(1);
                Logger::get_instance()->print_trace(".\n");
            }

            // step5: getting control config
            memset(ser_msg, 0, sizeof(ser_msg));
            Logger::get_instance()->print_trace_debug("IP: %s sendto get fse ctrl config ---> ", ser_list[i].c_str());
            fse->demo_ctrl_config_get(ser_list[i], NULL, ser_msg);
            Logger::get_instance()->print_trace_debug("%s\n", ser_msg);
            sleep(1);
        }
        usleep(1000); // The necessary delay for running multiple scripts
    }

    if (commandType["ctrl_config_save"].GetBool())
    {
        for (int i = 0; i < ser_num; i++)
        {
            memset(ser_msg, 0, sizeof(ser_msg));
            Logger::get_instance()->print_trace_debug("IP: %s sendto save ctrl_config ---> ", ser_list[i].c_str());
            fse->demo_ctrl_config_save(ser_list[i], NULL, ser_msg);
            Logger::get_instance()->print_trace_debug("%s\n", ser_msg);
        }
        usleep(1000); // The necessary delay for running multiple scripts
    }

    if (commandType["get_measured"].GetBool())
    {
        const rapidjson::Value &get_angel_num = param["get_measure_num"];
        uint64_t get_num = get_angel_num["get_angel_num"].GetInt64();
        while (get_num--)
        {
            for (int i = 0; i < ser_num; i++)
            {
                memset(ser_msg, 0, sizeof(ser_msg));
                Logger::get_instance()->print_trace_debug("IP: %s sendto get measured fse ---> ", ser_list[i].c_str());
                fse->demo_get_measured(ser_list[i], NULL, ser_msg);

                rapidjson::Document msg_json;
                if (msg_json.Parse(ser_msg).HasParseError())
                {
                    Logger::get_instance()->print_trace_error("fi_decode() failed\n");
                    return 0;
                }
                Logger::get_instance()->print_trace_debug("angle: %.9f, radian: %.9f\n", msg_json["angle"].GetDouble(), msg_json["radian"].GetDouble());
            }
        }
    }

    if (commandType["get_home_offset"].GetBool())
    {
        for (int i = 0; i < ser_num; i++)
        {
            memset(ser_msg, 0, sizeof(ser_msg));
            std::printf("IP: %s sendto get home offset fse ---> ", ser_list[i].c_str());
            fse->demo_home_offset_get(ser_list[i], NULL, ser_msg);

            rapidjson::Document msg_json;
            if (msg_json.Parse(ser_msg).HasParseError())
            {
                Logger::get_instance()->print_trace_error("fi_decode() failed\n");
                return 0;
            }
            std::printf("home_offset: %f\n", msg_json["home_offset"].GetFloat());
        }
        usleep(1000); // The necessary delay for running multiple scripts
    }

    if (commandType["set_home_offset"].GetBool())
    {
        for (int i = 0; i < ser_num; i++)
        {
            // step1: get home offset
            memset(ser_msg, 0, sizeof(ser_msg));
            std::printf("IP: %s sendto get home offset fse ---> ", ser_list[i].c_str());
            fse->demo_home_offset_get(ser_list[i], NULL, ser_msg);

            rapidjson::Document msg_json;
            if (msg_json.Parse(ser_msg).HasParseError())
            {
                Logger::get_instance()->print_trace_error("fi_decode() failed\n");
                return 0;
            }
            std::printf("home_offset: %f\n", msg_json["home_offset"].GetFloat());
            sleep(1);

            // step2: set home offset json
            const rapidjson::Value &home_offset_set_param = param["home_offset_set_param"];
            rapidjson::StringBuffer buffer;
            rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);

            writer.StartObject();
            writer.Key("method");
            writer.String("SET");

            writer.Key("reqTarget");
            writer.String("/home_offset");

            writer.Key("home_offset");
            writer.Double(home_offset_set_param["home_offset"].GetDouble());

            writer.EndObject();

            std::string home_offset_param = buffer.GetString();
            char send_set_config[1024] = {0};
            home_offset_param.copy(send_set_config, sizeof(send_set_config) - 1);
            send_set_config[sizeof(send_set_config) - 1] = '\0';

            // step3: set home offset
            memset(ser_msg, 0, sizeof(ser_msg));
            fse->demo_home_offset_set(fse->server_ip_filter[i], send_set_config, ser_msg);
            std::printf("%s\n", ser_msg);
            sleep(1);

            // step4: save control config params
            memset(ser_msg, 0, sizeof(ser_msg));
            std::printf("IP: %s sendto save home offset fse ---> ", ser_list[i].c_str());
            fse->demo_ctrl_config_save(ser_list[i], NULL, ser_msg);
            std::printf("%s\n", ser_msg);
            sleep(1);

            // step5: reboot fse
            memset(ser_msg, 0, sizeof(ser_msg));
            Logger::get_instance()->print_trace_warning("IP: %s sendto reboot fse ---> \n", ser_list[i].c_str());
            fse->demo_reboot_fse(ser_list[i], NULL, ser_msg);
            Logger::get_instance()->print_trace_debug("recv msg:%s\n", ser_msg);
            Logger::get_instance()->print_trace_warning("restarting ...\n");
            for (uint8_t i = 0; i < 7; i++)
            {
                sleep(1);
                Logger::get_instance()->print_trace(".\n");
            }

            // step6: get home offset
            memset(ser_msg, 0, sizeof(ser_msg));
            std::printf("IP: %s sendto get home offset fse ---> ", ser_list[i].c_str());
            fse->demo_home_offset_get(ser_list[i], NULL, ser_msg);
            std::printf("%s\n", ser_msg);
        }
        usleep(1000); // The necessary delay for running multiple scripts
    }

    if (commandType["set_home_position"].GetBool())
    {
        for (int i = 0; i < ser_num; i++)
        {
            memset(ser_msg, 0, sizeof(ser_msg));
            std::printf("IP: %s sendto set position fse ---> ", ser_list[i].c_str());
            fse->demo_home_position_set(ser_list[i], NULL, ser_msg);
            std::printf("%s\n", ser_msg);
        }
        usleep(1000); // The necessary delay for running multiple scripts
    }

    if (commandType["lookup_absEncoder"].GetBool())
    {
        Logger::get_instance()->print_trace("FSE server list:\n");
        for (int i = 0; i < ser_num; i++)
        {
            Logger::get_instance()->print_trace_debug("    %s\n", ser_list[i].c_str());
        }
    }

    if (commandType["lookup_ser"].GetBool())
    {
        std::string m_ser_list[254] = {""};
        memset(m_ser_list, 0, sizeof(m_ser_list));
        memcpy(m_ser_list, fse->server_ip, sizeof(m_ser_list));
        int m_ser_num = fse->server_ip_num;

        Logger::get_instance()->print_trace("FSE server list:\n");
        for (int i = 0; i < m_ser_num; i++)
        {
            Logger::get_instance()->print_trace_debug("    %s\n", m_ser_list[i].c_str());
        }
    }

    if (commandType["ota"].GetBool())
    {
        for (int i = 0; i < ser_num; i++)
        {
            memset(ser_msg, 0, sizeof(ser_msg));
            std::printf("IP: %s sendto ota fse ---> ", ser_list[i].c_str());
            fse->demo_ota(ser_list[i], NULL, ser_msg);
            std::printf("%s\n", ser_msg);
        }
        usleep(1000); // The necessary delay for running multiple scripts
    }

    if (commandType["ota_test"].GetBool())
    {
        for (int i = 0; i < ser_num; i++)
        {
            memset(ser_msg, 0, sizeof(ser_msg));
            std::printf("IP: %s sendto ota test fse ---> ", ser_list[i].c_str());
            fse->demo_ota_test(ser_list[i], NULL, ser_msg);
            std::printf("%s\n", ser_msg);
        }
        usleep(1000); // The necessary delay for running multiple scripts
    }

    if (commandType["ota_cloud"].GetBool())
    {
        for (int i = 0; i < ser_num; i++)
        {
            memset(ser_msg, 0, sizeof(ser_msg));
            std::printf("IP: %s sendto ota cloud fse ---> ", ser_list[i].c_str());
            fse->demo_ota_cloud(ser_list[i], NULL, ser_msg);
            std::printf("%s\n", ser_msg);
        }
        usleep(1000); // The necessary delay for running multiple scripts
    }

    if (commandType["reboot_fse"].GetBool())
    {
        for (int i = 0; i < ser_num; i++)
        {
            memset(ser_msg, 0, sizeof(ser_msg));
            std::printf("IP: %s sendto reboot fse ---> ", ser_list[i].c_str());
            fse->demo_reboot_fse(ser_list[i], NULL, ser_msg);
            std::printf("%s\n", ser_msg);
        }
        usleep(1000); // The necessary delay for running multiple scripts
    }

    if (commandType["reboot_ser"].GetBool())
    {
        std::string m_ser_list[254] = {""};
        memset(m_ser_list, 0, sizeof(m_ser_list));
        memcpy(m_ser_list, fse->server_ip, sizeof(m_ser_list));
        int m_ser_num = fse->server_ip_num;

        for (int i = 0; i < m_ser_num; i++)
        {
            memset(ser_msg, 0, sizeof(ser_msg));
            std::printf("IP: %s sendto reboot ---> ", m_ser_list[i].c_str());
            fse->demo_reboot(m_ser_list[i], NULL, ser_msg);
            std::printf("%s\n", ser_msg);
        }
        usleep(1000); // The necessary delay for running multiple scripts
    }
}