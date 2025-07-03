#include "magni_webots/MagniDriver.hpp"
#include <webots/motor.h>
#include <webots/robot.h>

namespace magni_driver {
  void MagniDriver::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
  {
    //get reference from webots
    right_motor = wb_robot_get_device("right wheel motor");
    left_motor = wb_robot_get_device("left wheel motor");
  }
  void MagniDriver::step()
  {

  }
}
