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
//#include "effects/builtin/filtereffect.h"


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

  MusicValuePublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("music_values", 1000);

  // Trigger for low pass filter.
  lpfTrigger_ = false;
  preLPFTrigger_ = false;

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
  if (edgeDetectionImageHistory.size() > 2) edgeDetectionImageHistory.erase(edgeDetectionImageHistory.begin()); // Hacked a 2 in here, history sizes will have no effect anymore
  edgeDetectionImageHistory.push_back(imageEdgeDetection);


  int totalDifferenceMusicValue = 0;

  for (unsigned int i = 0; i < imageEdgeDetection.data.size(); ++i){
    outputImage.data[4*i] = imageEdgeDetection.data[i];

    int totalDifference = 0;
    bool fast = false;

    for(unsigned int j = 0; j < edgeDetectionImageHistory.size()-1; ++j){

      // TODO: check normalization
      totalDifference += fabs(edgeDetectionImageHistory[j].data[i] - edgeDetectionImageHistory[j+1].data[i]);


//      if(j <= greenHistorySize_ && totalDifference >= greenIntensityThreshold_){
//        outputImage.data[4*i+1] = fmin(255, greenGain_ * totalDifference);
//      }
//        //outputImage.data[4*i+2] = fmin(255, 0.9 * totalDifference);
//        //fast = true;
//      if(j <= blueHistorySize_ && totalDifference >= blueIntensityThreshold_){
//        outputImage.data[4*i+2] = fmin(255, blueGain_ * totalDifference);
//      }

    }

    // For total Motion characterization for Music Filter adjustment.



    if (!fast){
      outputImage.data[4*i] = fmin(255, redGain_ * totalDifference);
      if (totalDifference <= redIntensityThreshold_) outputImage.data[4*i] = 0;
      if (outputImage.data[4*i] <= redIntensityThreshold_) outputImage.data[4*i] = 0;

      outputImage.data[4*i+1] = fmin(255, greenGain_ * totalDifference);
      if (totalDifference <= greenIntensityThreshold_) outputImage.data[4*i+1] = 0;
      if (outputImage.data[4*i+1] <= greenIntensityThreshold_) outputImage.data[4*i+1] = 0;

      outputImage.data[4*i+2] = fmin(255, blueGain_ * totalDifference);
      if (totalDifference <= blueIntensityThreshold_) outputImage.data[4*i+2] = 0;
      if (outputImage.data[4*i+2] <= blueIntensityThreshold_) outputImage.data[4*i+2] = 0;
    }

    int musicIntensityTreshold = 50;
    if (totalDifference > musicIntensityTreshold) totalDifferenceMusicValue += totalDifference;


    //ROS_INFO("Filter gain up: %f", lpfGainUp_);
    //ROS_INFO("Filter gain down: %f", this->lpfGainDown_);


    // Low pass filtering step.
    if (lpfTrigger_) {
      // Red
      if (outputImage.data[4*i] > outputImageTemp_.data[4*i])
        outputImage.data[4*i] = (1-redlpfGainUp_) * outputImage.data[4*i] + redlpfGainUp_ * outputImageTemp_.data[4*i];
      else
        outputImage.data[4*i] = (1-redlpfGainDown_) * outputImage.data[4*i] + redlpfGainDown_ * outputImageTemp_.data[4*i];
      //outputImage.data[4*i] = (1-lpfGain) * outputImage.data[4*i] + lpfGain * outputImageTemp_.data[4*i];

      // Green
      if (outputImage.data[4*i+1] > outputImageTemp_.data[4*i+1])
        outputImage.data[4*i+1] = (1-greenlpfGainUp_) * outputImage.data[4*i+1] + greenlpfGainUp_ * outputImageTemp_.data[4*i+1];
      else
        outputImage.data[4*i+1] = (1-greenlpfGainDown_) * outputImage.data[4*i+1] + greenlpfGainDown_ * outputImageTemp_.data[4*i+1];

      // Yellow
      if (outputImage.data[4*i+2] > outputImageTemp_.data[4*i+2])
        outputImage.data[4*i+2] = (1-bluelpfGainUp_) * outputImage.data[4*i+2] + bluelpfGainUp_ * outputImageTemp_.data[4*i+2];
      else
        outputImage.data[4*i+2] = (1-bluelpfGainDown_) * outputImage.data[4*i+2] + bluelpfGainDown_ * outputImageTemp_.data[4*i+2];

    }

  }




  // Helper image for low pass filtering.
  outputImageTemp_ = outputImage;
  lpfTrigger_ = true;

  coloredImagePublisher_.publish(outputImage);

  geometry_msgs::Twist musicTwist;

  if (quadraticCorrelation_) musicTwist.linear.x = max(min((float)totalDifferenceMusicValue * totalDifferenceMusicValue / (gainDivider_ * 1000000000000.0) - minusTerm_, 0.5) ,lowerBound_);
  else musicTwist.linear.x = max(min((float)totalDifferenceMusicValue / 15000000000.0 - 0.07, 0.5) ,0.0024);

  // Pre LPFing
  if (preLPFTrigger_ == true) musicTwist.linear.x = musicTwist.linear.x * (1.0 - preLPFMusicGain_) + oldMusicValue_ * preLPFMusicGain_;
  oldMusicValue_ = musicTwist.linear.x;
  preLPFTrigger_ = true;

  MusicValuePublisher_.publish(musicTwist);

  // Testing to call a Mixxx function.





}

void MotionVisualizer::drCallback(simple_kinect_motion_visualizer::VisualizationConfig &config, uint32_t level) {

  redHistorySize_ = config.redHistorySize;
  redGain_ = config.redGain;
  redIntensityThreshold_ = config.redIntensityThreshold;
  greenHistorySize_ = config.greenHistorySize;
  greenGain_ = config.greenGain;
  greenIntensityThreshold_ = config.greenIntensityThreshold;
  blueHistorySize_ = config.blueHistorySize;
  blueGain_ = config.blueGain;
  blueIntensityThreshold_ = config.blueIntensityThreshold;
  redlpfGainUp_ = config.redlpfGainUp;
  redlpfGainDown_ = config.redlpfGainDown;
  greenlpfGainUp_ = config.greenlpfGainUp;
  greenlpfGainDown_ = config.greenlpfGainDown;
  bluelpfGainUp_ = config.bluelpfGainUp;
  bluelpfGainDown_ = config.bluelpfGainDown;
  quadraticCorrelation_ = config.filterQuadraticCorrelation;
  gainDivider_ = config.musicGainDivider;
  minusTerm_ = config.minusTerm;
  lowerBound_ = config.lowerBound;
  preLPFMusicGain_ = config.preLPFMusicGain;
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
