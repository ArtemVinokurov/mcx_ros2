//
// Developer : Alexey Zakharov (alexey.zakharov@vectioneer.com)
// All rights reserved. Copyright (c) 2019 - 2022 VECTIONEER.
//
// This software is supplied under the terms of the MIT License, a
// copy of which should be located in the distribution where this
// file was obtained (LICENSE.txt).  A copy of the license may also be
// found online at https://opensource.org/licenses/MIT.
//

#include "helper.h"
#include <cstdio>
#include <iostream>
#include <list>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std;
using namespace mcx_cpp;

template <typename TimeT = std::chrono::milliseconds>
struct Measure {
  template <typename F, typename... Args>
  static typename TimeT::rep execution(F&& func, Args&&... args) {
    auto start = std::chrono::steady_clock::now();
    std::forward<decltype(func)>(func)(std::forward<Args>(args)...);
    auto duration = std::chrono::duration_cast<TimeT>(std::chrono::steady_clock::now() - start);
    return duration.count();
  }
};

void printStatus(const Status& status) {
  cerr << "Status code: " << (int)status.status() << endl;
  cerr << "Error code: " << status.error() << endl;
  cerr << "Error description: " << status.info() << endl;
}

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};





int main(int argc, char* argv[]) {

  clog << "Motorcortex C++ library version: " << mcx_cpp::version() << endl;

  // setting url
  std::cout << "Hi" << std::endl;
  std::string url = "wss://192.168.57.3:5568:5567";

  std::string shared_dir = ament_index_cpp::get_package_share_directory("mcx_ros2");
  ConnectionOptions options{shared_dir + "/resource/mcx.cert.pem"};
  std::cout << "Hi" << std::endl;
  if (argc > 1) {
    url = std::string(argv[1]);
  }

  // Creating a parameter tree;
  ParameterTree parameter_tree;
  Request req{parameter_tree};
  Subscribe sub{req};

  helper::connect(url, options, parameter_tree, req, sub);
  std::cout << "Hi" << std::endl;

  auto subscription = sub.subscribe({"root/Control_task/actual_cycle_max"}, "group1", 1000).get();

  if (subscription.status() == StatusCode::OK) {
    printf("Subscribed to a group\n");
  } else {
    printf("Failed to subscribe to a group\n");
    printStatus(subscription);
    throw(runtime_error(string("Failed to subscribe to a group: ").append(subscription.alias())));
  }

  subscription.notify([](const std::vector<GetParameter>& parameters) {
    for (auto& param : parameters) {
      double value{};
      time_t timestamp = param.value(value);
      printf("Notify: Timestamp: %lu Read from %s: %f\n", timestamp, param.path().c_str(), value);
    }
  });

  auto unsub = async([subscription, &sub]() {
    helper::sleep(5);
    auto res = sub.unsubscribe(subscription).get();
    if (subscription.status() == StatusCode::OK) {
      printf("Unsubscribed from the group\n");
    } else {
      printf("Failed to unsubscribe from the group\n");
      printStatus(subscription);
      throw(runtime_error(string("Failed to unsubscribe from the group: ").append(subscription.alias())));
    }
  });

  for (size_t i = 0; i < 10; i++) {
    auto parameters = subscription.read();
    for (auto& param : parameters) {
      double value{};
      time_t timestamp = param.value(value);
      printf("Poll: Timestamp: %lu Read from %s: %f\n", timestamp, param.path().c_str(), value);
    }
    this_thread::sleep_for(chrono::milliseconds(1000));
  }

  unsub.get();

  // Close connection
  sub.close();
  req.close();

  clog << "All tests have passed!" << endl;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;

}
