
#ifndef ARTIMETOROSTIMESTAMP_H
#define ARTIMETOROSTIMESTAMP_H

#include <ros/ros.h>

#ifdef ADEPT_PKG
#include "ariaUtil.h"
#else
#include "Aria/ariaUtil.h"
#endif

ros::Time convertArTimeToROS(const ArTime& t)
{
  // ARIA/ARNL times are in reference to an arbitrary starting time, not OS
  // clock, so find the time elapsed between now and t
  // to adjust the time stamp in ROS time vs. now accordingly.
  ArTime arianow;
  const double dtsec = (double) t.mSecSince(arianow) / 1000.0;
  //printf("was %f seconds ago\n", dtsec);
  return ros::Time(ros::Time::now().toSec() - dtsec);
}

#endif
