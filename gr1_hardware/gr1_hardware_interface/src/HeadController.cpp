#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <Fsa.h>
#include <FsaConfig.h>
#include <vector>
#include <string>
#include <thread>

// Json operation
#include <nlohmann/json.hpp>  
#include <fstream>     
       
#define PI 3.141592
// #define ESTIMATION_ONLY

class HeadController {
public:
    HeadController(){
        ros::NodeHandle nh_;
        
        std::vector<std::string> head_ip_list = {
            "192.168.137.93", 
            "192.168.137.94", 
            "192.168.137.95"  
        };
        
        // ================== set PID params ======================
        nh_.getParam("/motorlistFile", motorlistFile);
        ifstream ifs(motorlistFile);
        // Parse the JSON data
        nlohmann::json j;
        ifs >> j;

        std::vector<double> head_pos_gain; 
        std::vector<double> head_vel_gain;
        // get pid for head
        for (const std::string& ip : head_ip_list) {
            if (j.find(ip) != j.end()) {
                head_pos_gain.push_back(j[ip]["controlConfig"]["pos_gain"]);
                head_vel_gain.push_back(j[ip]["controlConfig"]["vel_gain"]);
            } else {
                // If the IP is not found in the JSON
                ROS_ERROR("An error occurred when reading motorlist.json.");
                exit(1);
            }
        }

        FSA_CONNECT::FSAConfig::FSAPIDParams pid_params;
        FSA_CONNECT::FSAConfig::FSAPIDParams get_pid_params;

        for (int i = 0; i < head_ip_list.size(); i++) { 
            head_fsa_motor_controllers[i].init(head_ip_list[i]);
            pid_params.control_position_kp = head_pos_gain[i];
            pid_params.control_velocity_kp = head_vel_gain[i];
            std::cout<<"write kp: "<<pid_params.control_position_kp << "kd:" << pid_params.control_velocity_kp << std::endl;
            head_fsa_motor_controllers[i].SetPIDParams(pid_params);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            head_fsa_motor_controllers[i].GetPIDParams(get_pid_params);
            std::cout<<"read kp: "<<get_pid_params.control_position_kp << "kd:" << get_pid_params.control_velocity_kp << std::endl;
        }
        // ================== END set PID params ======================
  
        #ifndef ESTIMATION_ONLY 
        for (int i = 0; i < head_ip_list.size(); i++) {
            head_fsa_motor_controllers[i].Enable();
            head_fsa_motor_controllers[i].EnablePosControl();
        }
        #endif
        
        head_joint_sub = nh_.subscribe("head_goals", 1, &HeadController::callback, this);
    }

    ~HeadController() {
    }

    void callback(const sensor_msgs::JointState::ConstPtr& msg) {
        target_joint_angles_[0] = command_angles_dir[0] * msg->position[0] * 180/PI; 
        target_joint_angles_[1] = command_angles_dir[1] * msg->position[0] * 180/PI; 
        target_joint_angles_[2] = command_angles_dir[2] * msg->position[1] * 180/PI; 

        for(int i=0; i<target_joint_angles_.size();i++){
            target_joint_angles_[i] = std::min(target_joint_angles_[i], max_angles[i]);
            target_joint_angles_[i] = std::max(target_joint_angles_[i], min_angles[i]);
        }

        std::vector<double> angle_error = {0.0, 0.0, 0.0};
        for(int i=0; i<init_joint_angles_.size();i++){
            double velocity, current;
            head_fsa_motor_controllers[i].GetPVC(init_joint_angles_[i], velocity, current);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            head_fsa_motor_controllers[i].GetPVC(init_joint_angles_[i], velocity, current);
            command_joint_angles_[i] = init_joint_angles_[i];
            angle_error[i] = target_joint_angles_[i] - init_joint_angles_[i];
        }
        double duration = std::max(*std::max_element(angle_error.begin(), angle_error.end()), -*std::min_element(angle_error.begin(), angle_error.end()))/max_velocity;
        std::cout<<duration<<std::endl;
        if(duration > 2*period){
            using clock = std::chrono::steady_clock;
            using milliseconds = std::chrono::milliseconds;

            auto start_time = clock::now();
            auto next_time = clock::now() + milliseconds(static_cast<int>(1000 / frequency));

            std::vector<double> delta_pos(init_joint_angles_.size());
            for (int i = 0; i < target_joint_angles_.size(); ++i) {
                delta_pos[i] = angle_error[i]/(frequency*duration);
            }

            while (true) {
                auto now = clock::now();
                if(now > start_time + milliseconds(int(1000*duration))){
                    break;
                }
                if (now < next_time) {
                    now = clock::now();
                    continue;
                }

                // Your loop code here
                for (int i = 0; i < target_joint_angles_.size(); ++i) {

                    command_joint_angles_[i] += delta_pos[i];
                    // std::cout<<i<<" "<<command_joint_angles_[i]<<std::endl;
                    head_fsa_motor_controllers[i].SetPosition(command_joint_angles_[i], 0, 0);
                }

                // Update the time for the next iteration
                next_time += milliseconds(static_cast<int>(period * 1000));
                // std::cout<<std::chrono::duration_cast<milliseconds>(next_time.time_since_epoch()).count()<<std::endl;
            }
        }
    }


private:
    ros::Subscriber head_joint_sub;
    ros::NodeHandle nh_;
    std::vector<double> init_joint_angles_ = {0.0, 0.0, 0.0};
    std::vector<double> command_joint_angles_ = {0.0, 0.0, 0.0};
    std::vector<double> target_joint_angles_  = {0.0, 0.0, 0.0};
    std::vector<double> command_angles_dir = {-1.0, 1.0, -1.0};
    std::vector<double> min_angles = { -50.0,  0.0, -45.0};  // degree
    std::vector<double> max_angles = { 0.0, 50.0, 45.0}; // degree
    std::vector<FSA_CONNECT::FSA> head_fsa_motor_controllers = std::vector<FSA_CONNECT::FSA>(3);
    const double max_velocity = 15.0;
    const double frequency = 500; // Hz
    const double period = 1.0 / frequency;
    std::string motorlistFile;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gr1_head_controller");

    HeadController headController;

    ros::spin();

    return 0;
}
