#pragma once

// #include <mcx-cpp/mcx-cpp.h>
#include "helper.h"

// #include "motion_program.h"
#include <mcx-robot-control/motion_program.h>
#include <mcx-robot-control/robot_command.h>
#include <mcx-robot-control/pose_transformer.h>
#include <mcx-robot-control/pose_ostream.h>
#include <ament_index_cpp/get_package_share_directory.hpp>




class RobotCommander
{
    public:

        explicit RobotCommander(const std::string& ip)
        {
            url = "wss://" + ip + ":5568:5567";
            std::string shared_dir = ament_index_cpp::get_package_share_directory("robot_bringup");
            path_to_cert = shared_dir + "/resource/mcx.cert.pem";
        }

        ~RobotCommander()
        {}

    private:
        void init() 
        {
            mcx_cpp::ConnectionOptions options{path_to_cert};
            helper::connect(url, options, parameter_tree, req, sub);
        }


        void enableRobot()
        {

        }

        void disableRobot()
        {
            
        }

        bool isEnabled()
        {

        }


    private:
        std::string url;
        std::string path_to_cert;
        mcx_cpp::ParameterTree parameter_tree;
        mcx_cpp::Request req{parameter_tree};
        mcx_cpp::Subscribe sub{req};

};