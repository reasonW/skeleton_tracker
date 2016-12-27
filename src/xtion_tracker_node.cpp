/**
 * \ref xtion_main.cpp
 *
 *  \date 24/giu/2015
 *  \author Alessio Levratti
 *  \version 1.0
 *  \bug
 *  \copyright GNU Public License.
 */

#include "xtion_tracker.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "xm_people");

  xtion_tracker* skeleton_tracker = new xtion_tracker();

  while (ros::ok())
  {
    skeleton_tracker->spinner();
  }

  delete skeleton_tracker;

  return 1;

}


