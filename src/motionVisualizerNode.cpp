/*
 * elevation_map_node.cpp
 *
 *  Created on: Oct 3, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>

#include <ros/console.h>

#include "simple_kinect_motion_visualizer/motionVisualizer.hpp"

int main(int argc, char** argv)
{
    {
    ros::init(argc, argv, "flower_core");
    ros::NodeHandle nodeHandle("~");
    //elevation_mapping::ElevationMapping elevationMap(nodeHandle);

    MotionVisualizer MotionVisualizer(nodeHandle);

    ROS_INFO("Hello du da");

    // Added by timon to set the logger level.
    //if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
    //   ros::console::notifyLoggerLevelsChanged();
    //}

    // Spin
    ros::AsyncSpinner spinner(2); // Use n threads // MANIPULATED!!!!!
    spinner.start();

    ros::waitForShutdown();
    }
  return 0;
}
