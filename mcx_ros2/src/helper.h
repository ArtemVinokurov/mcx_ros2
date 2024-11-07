//
// Developer : Alexey Zakharov (alexey.zakharov@vectioneer.com)
// All rights reserved. Copyright (c) 2019 - 2022 VECTIONEER.
//
// This software is supplied under the terms of the MIT License, a
// copy of which should be located in the distribution where this
// file was obtained (LICENSE.txt).  A copy of the license may also be
// found online at https://opensource.org/licenses/MIT.
//

#ifndef MOTORCORTEX_ROBOT_CONTROL_HELPER_H
#define MOTORCORTEX_ROBOT_CONTROL_HELPER_H

#include <mcx-cpp/mcx-cpp.h>
#include <iostream>
#include <string>
#include <thread>

namespace helper {

void sleep(double sec) { std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long>(sec * 1000))); }

void printStatus(const mcx_cpp::Status& status) {
  std::cerr << "Status code: " << (int)status.status() << std::endl;
  std::cerr << "Error code: " << status.error() << std::endl;
  std::cerr << "Error description: " << status.info() << std::endl;
}

void connect(const std::string& url, const mcx_cpp::ConnectionOptions& options, mcx_cpp::ParameterTree& parameter_tree,
             mcx_cpp::Request& req, mcx_cpp::Subscribe& sub) {
  using namespace mcx_cpp;

  size_t end;
  size_t start;
  std::string req_port;
  std::string sub_port;
  std::string address;
  try {
    end = url.find_last_of(':');
    start = url.rfind(':', end - 1);
    req_port = url.substr(start, url.length() - end);
    sub_port = url.substr(end);
    address = url.substr(0, start);
  } catch (...) {
    throw(std::runtime_error(std::string("Wrong connection URL format.\n"
                                         "Expected format: ws://address:request_port:subscribe_port.\n"
                                         "Example: ws://192.168.2.100:5558:5557")));
  }

  // Connecting to Req/Rep
  auto req_connection_done = req.connect(address + req_port, options);
  auto req_connection = req_connection_done.get();
  if (req_connection.status() == StatusCode::OK) {
    std::clog << "REQ connected to the server: " << req.url() << std::endl;
  } else {
    printStatus(req_connection);
    throw(std::runtime_error(std::string("REQ failed to connect to the server: ").append(req.url())));
  }

  // Sending login request
  auto login_done = req.login("root", "vectioneer");
  auto login_msg = login_done.get();
  if (login_msg.status() == StatusCode::OK) {
    std::clog << "Login to the server successful: " << req.url() << std::endl;
  } else {
    printStatus(login_msg);
    throw(std::runtime_error(std::string("Failed to login to the server: ").append(req.url())));
  }

  // Sending get parameter tree request
  auto parameter_tree_done = req.getParameterTree();
  auto parameter_tree_msg = parameter_tree_done.get();
  if (parameter_tree_msg.status() == StatusCode::OK) {
    parameter_tree.load(parameter_tree_msg);
    std::clog << "Received parameter tree message: " << req.url() << std::endl;
  } else {
    printStatus(parameter_tree_msg);
    throw(std::runtime_error("Failed to receive parameter tree message"));
  }

  // Connecting to Sub/Pub
  auto sub_connection_done = sub.connect(address + sub_port, options);
  auto sub_connection = sub_connection_done.get();
  if (sub_connection.status() == StatusCode::OK) {
    std::clog << "SUB connected to the server: " << sub.url() << std::endl;
  } else {
    printStatus(sub_connection);
    throw(std::runtime_error(std::string("Failed to connect SUB to the server: ").append(sub.url())));
  }
}

void connect(const std::string& url, mcx_cpp::ParameterTree& parameter_tree, mcx_cpp::Request& req,
             mcx_cpp::Subscribe& sub) {
  connect(url, {}, parameter_tree, req, sub);
}

} // namespace helper
#endif // MOTORCORTEX_ROBOT_CONTROL_HELPER_H
