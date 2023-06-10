#include "nutils.h"

#include <ros/ros.h>

void TopicAdvertisedTip(const char *topic) {
  ROS_INFO("%s has been advertised,use 'rostopic "
           "echo /%s' to view the data",
           topic, topic);
}
