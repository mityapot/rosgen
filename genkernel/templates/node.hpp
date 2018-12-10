/* [pkg_name] package
** [class_name] class header file
** Created on [date] by [author]
*/

#ifndef [preproc_dir]
#define [preproc_dir]

#include <exception>
#include <string>
#include <ros/ros.h>


class [class_name] {
public:
  [class_name](ros::NodeHandle& n);
  ~[class_name]();
protected:
  ros::NodeHandle _node;
};

#endif // [preproc_dir]
