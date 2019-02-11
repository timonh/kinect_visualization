/*
 */

// PCL
//#include <pcl/conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/PCLPointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/filters/voxel_grid.h>

//// Kindr
//#include <kindr/Core>
//#include <kindr_ros/kindr_ros.hpp>

//// Boost
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>

// STL
#include <string>
#include <math.h>
#include <limits>
#include "ros/ros.h"
#include "std_msgs/String.h"


#include "simple_kinect_motion_visualizer/motionVisualizer.hpp"

using namespace std;
//using namespace grid_map;
//using namespace ros;
//using namespace tf;
//using namespace pcl;
//using namespace kindr;
//using namespace kindr_ros;


MotionVisualizer::MotionVisualizer(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  ROS_INFO("Motion visualization node started.");
//  readParameters();
  edgeDetectionImageSubscriber_ = nodeHandle_.subscribe("/edge_detection/image", 1, &MotionVisualizer::edgeDetectionImageCallback, this);

  coloredImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>("/colored_image", 1000);

  // Trigger for low pass filter.
  lpfTrigger_ = false;

}


void MotionVisualizer::edgeDetectionImageCallback(
    const sensor_msgs::Image& imageEdgeDetection)
{
  //ROS_INFO("Image encoding: %s", imageEdgeDetection.encoding.c_str());
  //ROS_INFO("200 entry data: %d", imageEdgeDetection.data[1000]);

  sensor_msgs::Image outputImage;
  outputImage.encoding = "rgba8";
  outputImage.width = imageEdgeDetection.width;
  outputImage.height = imageEdgeDetection.height;
  outputImage.step = imageEdgeDetection.step * 4;
  outputImage.is_bigendian = imageEdgeDetection.is_bigendian;
  outputImage.header = imageEdgeDetection.header;
  outputImage.data.resize(4*imageEdgeDetection.data.size());

  //cout << typeid(imageEdgeDetection.data).name() << endl;

  // Set number of considered images for blurring
  if (edgeDetectionImageHistory.size() > redHistorySize_) edgeDetectionImageHistory.erase(edgeDetectionImageHistory.begin());
  edgeDetectionImageHistory.push_back(imageEdgeDetection);


  for (unsigned int i = 0; i < imageEdgeDetection.data.size(); ++i){
    outputImage.data[4*i] = imageEdgeDetection.data[i];

    int totalDifference = 0;
    bool fast = false;
    for(unsigned int j = 0; j < edgeDetectionImageHistory.size()-1; ++j){

      // TODO: check normalization
      totalDifference += fabs(edgeDetectionImageHistory[j].data[i] - edgeDetectionImageHistory[j+1].data[i]);


      if(j <= greenHistorySize_ && totalDifference >= greenIntensityThreshold_){
        outputImage.data[4*i+1] = fmin(255, greenGain_ * totalDifference);
      }
        //outputImage.data[4*i+2] = fmin(255, 0.9 * totalDifference);
        //fast = true;
      if(j <= blueHistorySize_ && totalDifference >= blueIntensityThreshold_){
        outputImage.data[4*i+2] = fmin(255, blueGain_ * totalDifference);
      }

    }
    if (!fast){
      outputImage.data[4*i] = fmin(255, redGain_ * totalDifference);
      if (totalDifference <= redIntensityThreshold_) outputImage.data[4*i] = 0;
      if (outputImage.data[4*i] <= redIntensityThreshold_) outputImage.data[4*i] = 0;

      if (totalDifference <= blueIntensityThreshold_) outputImage.data[4*i+2] = 0;
      if (outputImage.data[4*i+2] <= blueIntensityThreshold_) outputImage.data[4*i+2] = 0;
    }

    //ROS_INFO("Filter gain up: %f", lpfGainUp_);
    //ROS_INFO("Filter gain down: %f", this->lpfGainDown_);


    // Low pass filtering step.
    //double lpfGainUp = 0.2;
    double lpfGainUp = lpfGainUp_;

    //double lpfGainDown = 0.4;
    double lpfGainDown = lpfGainDown_;
    if (lpfTrigger_) {
      // Red
      if (outputImage.data[4*i] > outputImageTemp_.data[4*i])
        outputImage.data[4*i] = (1-lpfGainUp) * outputImage.data[4*i] + lpfGainUp * outputImageTemp_.data[4*i];
      else
        outputImage.data[4*i] = (1-lpfGainDown) * outputImage.data[4*i] + lpfGainDown * outputImageTemp_.data[4*i];
      //outputImage.data[4*i] = (1-lpfGain) * outputImage.data[4*i] + lpfGain * outputImageTemp_.data[4*i];

      // Yellow
      if (outputImage.data[4*i+1] > outputImageTemp_.data[4*i+1])
        outputImage.data[4*i+1] = (1-lpfGainUp) * outputImage.data[4*i+1] + lpfGainUp * outputImageTemp_.data[4*i+1];
      else
        outputImage.data[4*i+1] = (1-lpfGainDown) * outputImage.data[4*i+1] + lpfGainDown * outputImageTemp_.data[4*i+1];

    }

  }

  // Helper image for low pass filtering.
  outputImageTemp_ = outputImage;
  lpfTrigger_ = true;

  coloredImagePublisher_.publish(outputImage);
}

void MotionVisualizer::drCallback(simple_kinect_motion_visualizer::VisualizationConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f",
            config.lpfGainUp);

  redHistorySize_ = config.redHistorySize;
  redGain_ = config.redGain;
  redIntensityThreshold_ = config.redIntensityThreshold;
  greenHistorySize_ = config.greenHistorySize;
  greenGain_ = config.greenGain;
  greenIntensityThreshold_ = config.greenIntensityThreshold;
  blueHistorySize_ = config.blueHistorySize;
  blueGain_ = config.blueGain;
  blueIntensityThreshold_ = config.blueIntensityThreshold;
  lpfGainUp_ = config.lpfGainUp;
  lpfGainDown_ = config.lpfGainDown;
}


MotionVisualizer::~MotionVisualizer()
{

  //cout << "In destructor: " << this->lpfGainUp_ << endl;
//  // New added to write data to file for learning.
//  map_.writeDataFileForParameterLearning();

//  fusionServiceQueue_.clear();
//  fusionServiceQueue_.disable();
//  nodeHandle_.shutdown();
//  fusionServiceThread_.join();
}
