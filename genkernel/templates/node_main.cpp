/* [pkg_name] package
** [pkg_name] node main file
** Created on [date] by [author]
*/

#include <exception>
#include <iostream>

#include <[flname].hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "[node_name]");
  ros::NodeHandle node("~");
  ROS_INFO("[[node_name]] [node_name] is running");
  try {
    [class_name] obj(node);
  }
  catch (const std::logic_error& e) {
    ROS_ERROR("[[node_name]] %s", e.what());
  }
  catch (...) {
    std::cout << "[[node_name]] unexpected error" << std::endl;
  }
  std::cout << "[[node_name]] [node_name] is terminating" << std::endl;

  return 0;
};
