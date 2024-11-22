#ifndef ROBOT_DRIVER__HARDWARE_INTERFACE_HPP
#define ROBOT_DRIVER__HARDWARE_INTERFACE_HPP

#include "helper.h"
#include <mcx-robot-control/motion_program.h>
#include <mcx-robot-control/robot_command.h>
#include <mcx-robot-control/pose_transformer.h>
#include <mcx-robot-control/pose_ostream.h>
#include <mcx-robot-control/system_defs.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/visibility_control.h"


#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace robot_driver
{

class MCXHardwareInterface : public hardware_interface::SystemInterface
{


public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MCXHardwareInterface)
    ~MCXHardwareInterface();
    
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info);
    std::vector<hardware_interface::StateInterface> export_state_interfaces();
    std::vector<hardware_interface::CommandInterface> export_command_interfaces();

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state);
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state);
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state);
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state);



    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period);
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period);

    // hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
    //                                                             const std::vector<std::string>& stop_interfaces);
    
    // hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
    //                                                             const std::vector<std::string>& stop_interfaces);
    

protected:
    void readData();
    void printStatus(const mcx_cpp::Status& status);
    mcx_robot_control::States getRobotState();
    mcx_robot_control::Modes getRobotMode();


private:
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_position_commands_;
    std::vector<double> joint_position_commands_old_;
    std::vector<double> joint_velocities_commands_;

    mcx_robot_control::States robot_state_;
    mcx_robot_control::Modes robot_mode_;

    mcx_cpp::ParameterTree parameter_tree;
    mcx_cpp::Request req{parameter_tree};
    mcx_cpp::Subscribe sub{req};
    std::unique_ptr<mcx_robot_control::RobotCommand> robot_; 
    std::unique_ptr<mcx_cpp::Subscription> subscription_;

    bool first_pass_;
    bool initialized_;


};

}



#endif