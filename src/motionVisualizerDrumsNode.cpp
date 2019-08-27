/*
 * elevation_map_node.cpp
 *
 *  Created on: Oct 3, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <ros/ros.h>

#include <ros/console.h>

#include "simple_kinect_motion_visualizer/motionVisualizerDrums.hpp"

//#include <effects/builtin/filtereffect.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_to_drums");
  ros::NodeHandle nodeHandle("~");
  MotionVisualizerDrums mv(nodeHandle);

  // Added by timon to set the logger level.
  //if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
  //   ros::console::notifyLoggerLevelsChanged();
  //}

  // Dynamic Reconfigure.
  dynamic_reconfigure::Server<simple_kinect_motion_visualizer::VisualizationConfig> server;
  dynamic_reconfigure::Server<simple_kinect_motion_visualizer::VisualizationConfig>::CallbackType f;

  f = boost::bind(&MotionVisualizerDrums::drCallback, &mv, _1, _2);

  server.setCallback(f);

  //AutomaticFilter af;
  //af.testFunction();

  // Spin
  ros::AsyncSpinner spinner(2); // Use n threads // MANIPULATED!!!!!
  spinner.start();

  ros::waitForShutdown();


  return 0;
}
