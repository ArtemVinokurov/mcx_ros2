#include "robot_driver/hardware_interface.hpp"
#include <iostream>
#include <thread>

namespace robot_driver
{

MCXHardwareInterface::~MCXHardwareInterface()
{
    on_cleanup(rclcpp_lifecycle::State());
}



void MCXHardwareInterface::printStatus(const mcx_cpp::Status& status)
{
  RCLCPP_ERROR(rclcpp::get_logger("MCXHardwareInterface"), "Status code: %i", (int)status.status());
  RCLCPP_ERROR(rclcpp::get_logger("MCXHardwareInterface"), "Error code: %i", (int)status.status());
  RCLCPP_ERROR(rclcpp::get_logger("MCXHardwareInterface"), "Error description: %i", (int)status.status());
}


hardware_interface::CallbackReturn MCXHardwareInterface::on_init(const hardware_interface::HardwareInfo& system_info)
{
    if (hardware_interface::SystemInterface::on_init(system_info) != hardware_interface::CallbackReturn::SUCCESS) 
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = system_info;

  joint_positions_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  joint_velocity_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  joint_position_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  robot_state_ = mcx_robot_control::States::OFF_S;
  robot_mode_ = mcx_robot_control::Modes::PAUSE_M;


  return hardware_interface::CallbackReturn::SUCCESS;

}


hardware_interface::CallbackReturn MCXHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("MCXHardwareInterface"), "Starting robot driver, wait...");

    const std::string robot_ip = info_.hardware_parameters["robot_ip"];

    // connecting to the robot
    std::string url = "wss://" + robot_ip + ":5568:5567";
    std::string shared_dir = ament_index_cpp::get_package_share_directory("robot_driver");
    std::string path_to_cert = shared_dir + "/resource/mcx.cert.pem";

    mcx_cpp::ConnectionOptions options{path_to_cert};

    helper::connect(url, options, parameter_tree, req, sub);
    sub.subscribe({"root/logger/logOut"}, "log", 1).get().notify([](auto params) {
        std::string log;
        params[0].value(log);
        RCLCPP_INFO(rclcpp::get_logger("MCXHardwareInterface"), "Log: %s", log.c_str());
    });
    robot_ = std::make_unique<mcx_robot_control::RobotCommand>(req);
    robot_->acknowledge();
    robot_->engage();
    robot_->reset();
    
    // switch to joint mode 
    constexpr double timeout_sec = 5000;
    auto joint_done = robot_->manualJointMode(timeout_sec).get();
    if (joint_done.status() == mcx_cpp::StatusCode::OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("MCXHardwareInterface"), "Robot is in Manual Joint Mode");
    }
    else 
    {
        printStatus(joint_done);
        throw(std::runtime_error(std::string("Failed to set Manual Joint Mode")));
    }

    
}   

void MCXHardwareInterface::readData()
{

}


}
