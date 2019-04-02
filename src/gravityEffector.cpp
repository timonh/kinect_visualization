/*
 */

//// Boost
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>

// STL
#include <string>
#include <math.h>
#include <limits>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "simple_kinect_motion_visualizer/gravityEffector.hpp"

using namespace std;
//using namespace grid_map;
//using namespace ros;
//using namespace tf;
//using namespace pcl;
//using namespace kindr;
//using namespace kindr_ros;


GravityEffector::GravityEffector(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  ROS_INFO("Gravity effects node started.");

  std::string topic_name;
  bool depth_image = false;
  if (depth_image) topic_name = "/camera/depth/image_raw";
  else topic_name = "/camera/rgb/image_raw";

  inputImageSubscriber_ = nodeHandle_.subscribe(topic_name, 1, &GravityEffector::inputImageCallback, this);

  outputImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>("/gravity_output_image", 1000);

  cogSubscriber_ = nodeHandle_.subscribe("/motionVisualizerNode/cog_keyvalues", 1, &GravityEffector::cogCallback, this);

  //coloredCombinedImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>("/colored_image_combined", 1000);

  //MusicValuePublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("music_values", 1000);

  //cogPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("cog_keyvalues", 1000);

  // Trigger for low pass filter.
  //lpfTrigger_ = false;
  //preLPFTrigger_ = false;

  // Bool to chose if generating combined image
  //generateCombinedImage_ = true;

}


void GravityEffector::inputImageCallback(
    const sensor_msgs::Image& inputImage)
{
  //ROS_INFO("Image encoding: %s", imageEdgeDetection.encoding.c_str());
  //ROS_INFO("200 entry data: %d", imageEdgeDetection.data[1000]);

  sensor_msgs::Image outputImage;
  outputImage.encoding = "rgba8";
  outputImage.width = inputImage.width;
  outputImage.height = inputImage.height;
  outputImage.step = inputImage.step * 4;
  outputImage.is_bigendian = inputImage.is_bigendian;
  outputImage.header = inputImage.header;
  outputImage.data.resize(4*inputImage.data.size());


//  sensor_msgs::Image outputImageCombined;
//  outputImageCombined.encoding = "rgba8";
//  outputImageCombined.width = imageEdgeDetection.width;
//  outputImageCombined.height = imageEdgeDetection.height;
//  outputImageCombined.step = imageEdgeDetection.step * 4;
//  outputImageCombined.is_bigendian = imageEdgeDetection.is_bigendian;
//  outputImageCombined.header = imageEdgeDetection.header;
//  outputImageCombined.data.resize(4*imageEdgeDetection.data.size());

  //cout << typeid(imageEdgeDetection.data).name() << endl;

  // Set number of considered images for blurring
  //if (edgeDetectionImageHistory.size() > 2) edgeDetectionImageHistory.erase(edgeDetectionImageHistory.begin()); // Hacked a 2 in here, history sizes will have no effect anymore
  //edgeDetectionImageHistory.push_back(imageEdgeDetection);


  //int totalDifferenceMusicValue = 0;

  // Fot calculation of pixel center of gravity:
  //int totalDifferenceTimesHorizontalPixels;
  //int totalDOfferenceTimesVertialPixels;

  ///std::cout << "Size of the Edge detection image: " << imageEdgeDetection.data.size() << std::endl;

  //imageEdgeDetection.header.

  // For center of gravity calculation.
  //int accumulatedTotalDifference = 0;
  //double accumulatedTotalDifferenceTimesXAxis = 0.0;
  //double accumulatedTotalDifferenceTimesYAxis = 0.0;



  // Get some info about the input image.
  //std::cout << "Input Image Type" << inputImage.encoding << std::endl;
  //std::cout << "Input Image Width" << inputImage.width << std::endl;
  //std::cout << "Input Image Height" << inputImage.height << std::endl;

  for (unsigned int i = 0; i < inputImage.data.size(); ++i){
    //outputImage.data[4*i] = inputImage.data[i];



  }

  outputImagePublisher_.publish(outputImage);


}

void GravityEffector::cogCallback(geometry_msgs::Twist cogKeyvalues)
{
    std::cout << "cogKeys: x " << cogKeyvalues.linear.x << std::endl;
    cogX_ = cogKeyvalues.linear.x;
    cogY_ = cogKeyvalues.linear.y;
    accumulatedDifference_ = cogKeyvalues.linear.z;
}

//void MotionVisualizer::drCallback(simple_kinect_motion_visualizer::VisualizationConfig &config, uint32_t level) {

//  redHistorySize_ = config.redHistorySize;
//  redGain_ = config.redGain;
//  redIntensityThreshold_ = config.redIntensityThreshold;
//  greenHistorySize_ = config.greenHistorySize;
//  greenGain_ = config.greenGain;
//  greenIntensityThreshold_ = config.greenIntensityThreshold;
//  blueHistorySize_ = config.blueHistorySize;
//  blueGain_ = config.blueGain;
//  blueIntensityThreshold_ = config.blueIntensityThreshold;
//  redlpfGainUp_ = config.redlpfGainUp;
//  redlpfGainDown_ = config.redlpfGainDown;
//  greenlpfGainUp_ = config.greenlpfGainUp;
//  greenlpfGainDown_ = config.greenlpfGainDown;
//  bluelpfGainUp_ = config.bluelpfGainUp;
//  bluelpfGainDown_ = config.bluelpfGainDown;
//  quadraticCorrelation_ = config.filterQuadraticCorrelation;
//  gainDivider_ = config.musicGainDivider;
//  minusTerm_ = config.minusTerm;
//  lowerBound_ = config.lowerBound;
//  preLPFMusicGain_ = config.preLPFMusicGain;
//}


GravityEffector::~GravityEffector()
{

  //cout << "In destructor: " << this->lpfGainUp_ << endl;
//  // New added to write data to file for learning.
//  map_.writeDataFileForParameterLearning();

//  fusionServiceQueue_.clear();
//  fusionServiceQueue_.disable();
//  nodeHandle_.shutdown();
//  fusionServiceThread_.join();
}
