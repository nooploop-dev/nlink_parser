#include "nutils.h"
#include <ros/ros.h>

void topicadvertisedTip(const char *topic) {
  ROS_INFO("%s has been advertised,use 'rostopic "
           "echo /%s' to view the data",
           topic, topic);
}
