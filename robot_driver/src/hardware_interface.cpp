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


hardware_interface::CallbackReturn MCXHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("MCXHardwareInterface"), "Activating HW interface");
    return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::CallbackReturn MCXHardwareInterface::on_init(const hardware_interface::HardwareInfo& system_info)
{
    if (hardware_interface::SystemInterface::on_init(system_info) != hardware_interface::CallbackReturn::SUCCESS) 
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = system_info;

  joint_positions_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  joint_velocities_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  joint_position_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  joint_velocities_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  robot_ = nullptr;
  robot_state_ = mcx_robot_control::States::OFF_S;
  robot_mode_ = mcx_robot_control::Modes::PAUSE_M;
  first_pass_ = true;
  initialized_ = false;
  
  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("MCXHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) 
    {
        RCLCPP_FATAL(rclcpp::get_logger("MCXHardwareInterface"), "Joint '%s' has %zu state interface. 2 expected.",
                    joint.name.c_str(), joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MCXHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
        
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MCXHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_commands_[i]));
    }

    return command_interfaces;
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

    subscription_ = std::make_unique<mcx_cpp::Subscription>(sub.subscribe({"root/ManipulatorControl/jointPositionsActual", 
                                                                           "root/ManipulatorControl/jointVelocitiesActual"}, "group1", 1).get());

    if (subscription_->status() == mcx_cpp::StatusCode::OK) 
    {
        RCLCPP_INFO(rclcpp::get_logger("MCXHardwareInterface"), "Subscribed to a tree parameters");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("MCXHardwareInterface"), "Failed to subscribe to a tree parameters");
        printStatus(*subscription_);
    }

      RCLCPP_INFO(rclcpp::get_logger("MCXHardwareInterface"), "System successfully started!");

      return hardware_interface::CallbackReturn::SUCCESS;
}


mcx_robot_control::States MCXHardwareInterface::getRobotState()
{
    return robot_->getState();
}


mcx_robot_control::Modes MCXHardwareInterface::getRobotMode()
{
    return robot_->getMode();
}

hardware_interface::return_type MCXHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    auto parameters = subscription_->read();
    time_t timestamp{0};
    for (auto& param : parameters) {
        std::string param_path = param.path().c_str();
        if (param_path == "root/ManipulatorControl/jointPositionsActual") {
            timestamp = param.value(joint_positions_);
        }
        else if (param_path == "root/ManipulatorControl/jointVelocitiesActual") {
            timestamp = param.value(joint_velocities_);
        }
    }

    robot_state_ = getRobotState();
    robot_mode_ = getRobotMode();

    if(first_pass_ && !initialized_) {
        joint_position_commands_ = joint_position_commands_old_ = joint_positions_;
        joint_velocities_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        initialized_ = true;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MCXHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    mcx_cpp::StatusReply rep = req.setParameter("root/ManipulatorControl/hostInJointVelocity", joint_velocities_commands_);
}


hardware_interface::CallbackReturn MCXHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("MCXHardwareInterface"), "Stopping robot driver, please wait...");
    robot_->acknowledge();
    robot_->disengage();
    robot_.reset();

    req.close();
    sub.close();
    RCLCPP_INFO(rclcpp::get_logger("MCXHardwareInterface"), "System successfully stopped!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robot_driver::MCXHardwareInterface, hardware_interface::SystemInterface)