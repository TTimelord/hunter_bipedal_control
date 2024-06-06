#include "fse.h"

using namespace Predefine;
using namespace Utils;
using namespace Sensor;

FSE::FSE(/* args */) {}

FSE::~FSE() {}

void FSE::fi_init_network()
{
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;

    // left
    this->fse_socket = socket(AF_INET, SOCK_DGRAM, 0);
    short set_sock_opt_flg = setsockopt(this->fse_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    if (set_sock_opt_flg == -1)
    {
        Logger::get_instance()->print_trace_error("Error: fi_init_network() setsockopt() failed\n");
    }
    // COMM
    this->fse_sockaddr_in_comm.sin_family = AF_INET;
    this->fse_sockaddr_in_comm.sin_port = htons(SERVER_PORT_COMM);
    inet_pton(AF_INET, SERVER_IP, &this->fse_sockaddr_in_comm.sin_addr.s_addr);
    // CTRL
    this->fse_sockaddr_in_ctrl.sin_family = AF_INET;
    this->fse_sockaddr_in_ctrl.sin_port = htons(SERVER_PORT_CTRL);
    inet_pton(AF_INET, SERVER_IP, &this->fse_sockaddr_in_ctrl.sin_addr.s_addr);
}

void FSE::fi_init_fse()
{
    this->fi_init_network();
}

void FSE::fi_fse_comm(std::string ip, int port, char *msg, char *client_recv_msg)
{
    // this->fi_encode();
    this->fi_send_msg(ip, port, msg);
    usleep(400);
    this->fi_recv_msg(client_recv_msg);
}

void FSE::fi_decode(char *msg)
{
    // Logger::get_instance()->print_trace_debug("decode-> %s\n", msg);
    // usleep(100);
    if (this->msg_json.Parse(msg).HasParseError())
    {
        Logger::get_instance()->print_trace_error("decode failed\n");
        return;
    }
    switch (this->work_mode)
    {
    case WorkMode::NONE:
        break;
    case WorkMode::BROADCASE_FILTER_MODE:
    {
        if (this->msg_json["type"].GetString() == this->server_filter_type)
        {
            this->server_ip_filter[this->server_ip_filter_num] = this->msg_json["IP"].GetString();
            this->server_ip_filter_num++;
        }
        break;
    }
    case WorkMode::BROADCAST_MODE:
    {
        break;
    }
    case WorkMode::SIGLE_MODE:
    {
        break;
    }
    default:
        break;
    }
}

void FSE::fi_encode()
{
    Logger::get_instance()->print_trace("encode data\n");
}

void FSE::fi_send_msg(std::string ip, int port, char *msg)
{
    if (!this->is_init)
    {
        this->fi_init_fse();
        Logger::get_instance()->print_trace_debug("init fse\n");
        this->is_init = 1;
        usleep(20);
    }
    int ret = -1;

    memset(this->send_buff, 0, sizeof(this->send_buff));
    mempcpy(this->send_buff, msg, sizeof(this->send_buff));
    memset(this->server_ip_filter, 0, sizeof(this->server_ip_filter));
    memset(this->server_ip, 0, sizeof(this->server_ip));
    this->server_ip_filter_num = 0;
    this->server_ip_num = 0;
    Logger::get_instance()->print_trace_warning("send json-> %s\n", this->send_buff);

    switch (this->work_mode)
    {
    case WorkMode::NONE:
        /* code */
        break;
    case WorkMode::BROADCASE_FILTER_MODE:
    case WorkMode::BROADCAST_MODE:
    {
        while (1)
        {
            ret = sendto(this->fse_socket, this->send_buff, BUFFER_SIZE, 0, (struct sockaddr *)&this->fse_sockaddr_in_comm, sizeof(this->fse_sockaddr_in_comm));
            if (ret == -1)
            {
                Logger::get_instance()->print_trace_error("broadcast sendto() failed\n");
                usleep(10);
                continue;
            }
            break;
        }
        break;
    }
    case WorkMode::SIGLE_MODE:
    {
        this->work_ip = &*ip.begin();
        this->fse_sockaddr_in_work.sin_family = AF_INET;
        this->fse_sockaddr_in_work.sin_port = htons(port);
        inet_pton(AF_INET, this->work_ip, &this->fse_sockaddr_in_work.sin_addr.s_addr);
        usleep(1);
        ret = sendto(this->fse_socket, this->send_buff, BUFFER_SIZE, 0, (struct sockaddr *)&this->fse_sockaddr_in_work, sizeof(this->fse_sockaddr_in_work));
        if (ret == -1)
        {
            Logger::get_instance()->print_trace_error("sigle sendto() failed\n");
        }

        break;
    }
    default:
        break;
    }
}

void FSE::fi_recv_msg(char *client_recv_msg)
{
    int ret = -1;
    memset(this->recv_buff, 0, sizeof(this->recv_buff));

    switch (this->work_mode)
    {
    case WorkMode::NONE:
        break;
    case WorkMode::BROADCASE_FILTER_MODE:
    case WorkMode::BROADCAST_MODE:
    {
        this->server_ip_num = 0;
        while (1)
        {
            ret = recvfrom(this->fse_socket, this->recv_buff, sizeof(this->recv_buff), 0, (struct sockaddr *)&this->fse_sockaddr_in_recv, &this->sockaddr_len);
            if (ret == -1)
            {
                if (this->server_ip_num == 0)
                {
                    Logger::get_instance()->print_trace_error("broadcast recvfrom() failed\n");
                }
                else
                {
                    Logger::get_instance()->print_trace("broadcast recvfrom() successfully\n");
                }
                break;
            }
            else
            {
                char *ip_sigle = inet_ntoa(this->fse_sockaddr_in_recv.sin_addr);
                this->server_ip[this->server_ip_num] = ip_sigle;
                this->server_ip_num++;
                this->fi_decode(this->recv_buff);
            }
            memset(this->recv_buff, 0, sizeof(this->recv_buff));
        }
        break;
    }
    case WorkMode::SIGLE_MODE:
    {
        ret = recvfrom(this->fse_socket, this->recv_buff, sizeof(this->recv_buff), 0, NULL, NULL);
        if (ret == -1)
        {
            Logger::get_instance()->print_trace_error("sigle recvfrom() failed\n");
            break;
        }
        else
        {
            memcpy(client_recv_msg, this->recv_buff, sizeof(this->recv_buff));
        }
        break;
    }
    default:
        break;
    }
}

void FSE::demo_comm_config_get(std::string sigle_ip, char *define_msg_sendto, char *client_recv_msg)
{
    Logger::get_instance()->print_trace("demo_comm_config_get()\n");
    this->work_mode = WorkMode::SIGLE_MODE;
    this->fi_fse_comm(sigle_ip, SERVER_PORT_COMM, this->comm_config_get, client_recv_msg);
}

void FSE::demo_comm_config_set(std::string sigle_ip, char *define_msg_sendto, char *client_recv_msg)
{
    Logger::get_instance()->print_trace("demo_comm_config_set()\n");
    this->work_mode = WorkMode::SIGLE_MODE;
    this->fi_fse_comm(sigle_ip, SERVER_PORT_COMM, define_msg_sendto, client_recv_msg);
}

void FSE::demo_ctrl_config_get(std::string sigle_ip, char *define_msg_sendto, char *client_recv_msg)
{
    Logger::get_instance()->print_trace("demo_ctrl_config_get()\n");
    this->work_mode = WorkMode::SIGLE_MODE;
    this->fi_fse_comm(sigle_ip, SERVER_PORT_CTRL, this->ctrl_config_get, client_recv_msg);
}

void FSE::demo_ctrl_config_save(std::string sigle_ip, char *define_msg_sendto, char *client_recv_msg)
{
    Logger::get_instance()->print_trace("demo_ctrl_config_save()\n");
    this->work_mode = WorkMode::SIGLE_MODE;
    this->fi_fse_comm(sigle_ip, SERVER_PORT_CTRL, this->ctrl_config_save, client_recv_msg);
}

void FSE::demo_ctrl_config_set(std::string sigle_ip, char *define_msg_sendto, char *client_recv_msg)
{
    Logger::get_instance()->print_trace("demo_ctrl_config_set()\n");
    this->work_mode = WorkMode::SIGLE_MODE;
    this->fi_fse_comm(sigle_ip, SERVER_PORT_CTRL, define_msg_sendto, client_recv_msg);
}

void FSE::demo_get_measured(std::string sigle_ip, char *define_msg_sendto, char *client_recv_msg)
{
    Logger::get_instance()->print_trace("demo_get_measured()\n");
    this->work_mode = WorkMode::SIGLE_MODE;
    this->fi_fse_comm(sigle_ip, SERVER_PORT_CTRL, this->get_measured, client_recv_msg);
}

void FSE::demo_home_offset_get(std::string sigle_ip, char *define_msg_sendto, char *client_recv_msg)
{
    Logger::get_instance()->print_trace("demo_home_offset_get()\n");
    this->work_mode = WorkMode::SIGLE_MODE;
    this->fi_fse_comm(sigle_ip, SERVER_PORT_CTRL, this->get_home_offset, client_recv_msg);
}

void FSE::demo_home_offset_set(std::string sigle_ip, char *define_msg_sendto, char *client_recv_msg)
{
    Logger::get_instance()->print_trace("demo_home_offset_set()\n");
    this->work_mode = WorkMode::SIGLE_MODE;
    this->fi_fse_comm(sigle_ip, SERVER_PORT_CTRL, define_msg_sendto, client_recv_msg);
}

void FSE::demo_home_position_set(std::string sigle_ip, char *define_msg_sendto, char *client_recv_msg)
{
    Logger::get_instance()->print_trace("demo_home_position_set()\n");
    this->work_mode = WorkMode::SIGLE_MODE;
    this->fi_fse_comm(sigle_ip, SERVER_PORT_CTRL, this->set_home_position, client_recv_msg);
    // this->fi_fse_comm(sigle_ip, SERVER_PORT_CTRL, this->comm_config_get, client_recv_msg);
}

void FSE::demo_lookup_abs_encoder()
{
}

void FSE::demo_lookup()
{
}

void FSE::demo_ota_cloud(std::string sigle_ip, char *define_msg_sendto, char *client_recv_msg)
{
    Logger::get_instance()->print_trace("demo_ota\n");
    this->work_mode = WorkMode::SIGLE_MODE;
    this->fi_fse_comm(sigle_ip, SERVER_PORT_COMM, this->ota_cloud, client_recv_msg);
}

void FSE::demo_ota_test(std::string sigle_ip, char *define_msg_sendto, char *client_recv_msg)
{
    Logger::get_instance()->print_trace("demo_ota_test\n");
    this->work_mode = WorkMode::SIGLE_MODE;
    this->fi_fse_comm(sigle_ip, SERVER_PORT_COMM, this->ota_test, client_recv_msg);
}

void FSE::demo_ota(std::string sigle_ip, char *define_msg_sendto, char *client_recv_msg)
{
    Logger::get_instance()->print_trace("demo_ota\n");
    this->work_mode = WorkMode::SIGLE_MODE;
    this->fi_fse_comm(sigle_ip, SERVER_PORT_COMM, this->ota, client_recv_msg);
}

void FSE::demo_reboot_fse(std::string sigle_ip, char *define_msg_sendto, char *client_recv_msg)
{
    Logger::get_instance()->print_trace("demo_reboot_fse()\n");
    this->work_mode = WorkMode::SIGLE_MODE;
    this->fi_fse_comm(sigle_ip, SERVER_PORT_CTRL, this->reboot_fse, client_recv_msg);
}

void FSE::demo_reboot(std::string sigle_ip, char *define_msg_sendto, char *client_recv_msg)
{
    Logger::get_instance()->print_trace("demo_reboot()\n");
    this->work_mode = WorkMode::SIGLE_MODE;
    this->fi_fse_comm(sigle_ip, SERVER_PORT_CTRL, this->reboot_all, client_recv_msg);
}

void FSE::demo_broadcase()
{
    Logger::get_instance()->print_trace("broadcase all\n");
    this->work_mode = WorkMode::BROADCAST_MODE;
    this->fi_fse_comm(SERVER_IP, SERVER_PORT_COMM, this->broadcast_msg, NULL);
    Logger::get_instance()->print_trace("broadcase finished\n");
}

void FSE::demo_broadcase_filter(std::string filter_type)
{
    Logger::get_instance()->print_trace("broadcase all with filter\n");
    this->work_mode = WorkMode::BROADCASE_FILTER_MODE;
    this->server_filter_type = filter_type;
    this->fi_fse_comm(SERVER_IP, SERVER_PORT_COMM, this->broadcast_msg, NULL);
    Logger::get_instance()->print_trace("filter broadcase finished\n");
}

