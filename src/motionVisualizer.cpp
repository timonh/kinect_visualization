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

  std::string topic_name;
  bool depth_image = false;
  if (depth_image) topic_name = "/camera/depth/image_raw";
  else topic_name = "/edge_detection/image";

  edgeDetectionImageSubscriber_ = nodeHandle_.subscribe(topic_name, 1, &MotionVisualizer::edgeDetectionImageCallback, this);

  coloredImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>("/colored_image2", 1000);

  coloredCombinedImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>("/colored_image_combined2", 1000);

  MusicValuePublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("music_values", 1000);

  cogPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("cog_keyvalues", 1000);

  // Trigger for low pass filter.
  lpfTrigger_ = false;
  preLPFTrigger_ = false;

  // Bool to chose if generating combined image
  generateCombinedImage_ = false;

  // Bool to chose if calculating cog of motion
  getCOG_ = false;

  // Bool to chose if publishing music values
  publishMusicTwist_ = false;
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


  sensor_msgs::Image outputImageCombined;
  outputImageCombined.encoding = "rgba8";
  outputImageCombined.width = imageEdgeDetection.width;
  outputImageCombined.height = imageEdgeDetection.height;
  outputImageCombined.step = imageEdgeDetection.step * 4;
  outputImageCombined.is_bigendian = imageEdgeDetection.is_bigendian;
  outputImageCombined.header = imageEdgeDetection.header;
  outputImageCombined.data.resize(4*imageEdgeDetection.data.size());




  //cout << typeid(imageEdgeDetection.data).name() << endl;

  // Set number of considered images for blurring
  if (edgeDetectionImageHistory_.size() > 1) edgeDetectionImageHistory_.erase(edgeDetectionImageHistory_.begin()); // Hacked a 2 in here, history sizes will have no effect anymore
  edgeDetectionImageHistory_.push_back(imageEdgeDetection);

  if (outputImages_.size() > 1) outputImages_.erase(outputImages_.begin()); // Hacked a 2 in here, history sizes will have no effect anymore
  outputImages_.push_back(outputImage);

  //std::cout << "edgeDetectionImageHistory__size: " << edgeDetectionImageHistory_.size() << std::endl;

  int totalDifferenceMusicValue = 0;

  // Fot calculation of pixel center of gravity:
  //int totalDifferenceTimesHorizontalPixels;
  //int totalDOfferenceTimesVertialPixels;

  //std::cout << "Size of the Edge detection image: " << imageEdgeDetection.data.size() << std::endl;

  //imageEdgeDetection.header.

  // For center of gravity calculation.

  int accumulatedTotalDifference = 0;
  double accumulatedTotalDifferenceTimesXAxis = 0.0;
  double accumulatedTotalDifferenceTimesYAxis = 0.0;




  for (unsigned int i = 0; i < imageEdgeDetection.data.size(); ++i){
    //outputImage.data[4*i] = imageEdgeDetection.data[i];



    // Get location within image: x = 0: left border, x = 1 reight border, y = 0: top, y = 0: bottom


    double pixelXAxis = double(i % imageEdgeDetection.width) / (double)imageEdgeDetection.width;
    double pixelYAxis = double(floor(i / imageEdgeDetection.width)+1) / (double)imageEdgeDetection.height;
    //std::cout << "XAxis: " << pixelXAxis << std::endl;
    //std::cout << "YAxis: " << pixelYAxis << std::endl;



    int totalDifference = 0;
    bool fast = false;

    for(unsigned int j = 0; j < edgeDetectionImageHistory_.size()-1; ++j){

      //std::cout << "Went in here " << j+1 << " times" << std::endl;
      // TODO: check normalization
      totalDifference += fabs(edgeDetectionImageHistory_[j].data[i] - edgeDetectionImageHistory_[j+1].data[i]);


      // For calculation of the center of gravity of the changes:
      //int localDifference = fabs(edgeDetectionImageHistory_[j].data[i] - edgeDetectionImageHistory_[j+1].data[i]);




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





    int musicIntensityTreshold = 50;
    if (totalDifference > musicIntensityTreshold) totalDifferenceMusicValue += totalDifference;


    //ROS_INFO("Filter gain up: %f", lpfGainUp_);
    //ROS_INFO("Filter gain down: %f", this->lpfGainDown_);


    // Low pass filtering step.
    if (lpfTrigger_) {
        if (outputImages_.size() > 1) {

          if (!fast){
            outputImages_[1].data[4*i] = fmin(255, redGain_ * totalDifference);
            if (totalDifference <= redIntensityThreshold_) outputImages_[1].data[4*i] = 0;
            if (outputImages_[1].data[4*i] <= redIntensityThreshold_) outputImages_[1].data[4*i] = 0;

            outputImages_[1].data[4*i+1] = fmin(255, greenGain_ * totalDifference);
            if (totalDifference <= greenIntensityThreshold_) outputImages_[1].data[4*i+1] = 0;
            if (outputImages_[1].data[4*i+1] <= greenIntensityThreshold_) outputImages_[1].data[4*i+1] = 0;

            outputImages_[1].data[4*i+2] = fmin(255, blueGain_ * totalDifference);
            if (totalDifference <= blueIntensityThreshold_) outputImages_[1].data[4*i+2] = 0;
            if (outputImages_[1].data[4*i+2] <= blueIntensityThreshold_) outputImages_[1].data[4*i+2] = 0;
          }
          // Red
          if (outputImages_[1].data[4*i] > outputImages_[0].data[4*i])
            outputImages_[1].data[4*i] = (1-redlpfGainUp_) * outputImages_[1].data[4*i] + redlpfGainUp_ * outputImages_[0].data[4*i];
          else
            outputImages_[1].data[4*i] = (1-redlpfGainDown_) * outputImages_[1].data[4*i] + redlpfGainDown_ * outputImages_[0].data[4*i];
          //outputImage.data[4*i] = (1-lpfGain) * outputImage.data[4*i] + lpfGain * outputImageTemp_.data[4*i];

          // Green
          if (outputImages_[1].data[4*i+1] > outputImages_[0].data[4*i+1])
            outputImages_[1].data[4*i+1] = (1-greenlpfGainUp_) * outputImages_[1].data[4*i+1] + greenlpfGainUp_ * outputImages_[0].data[4*i+1];
          else
            outputImages_[1].data[4*i+1] = (1-greenlpfGainDown_) * outputImages_[1].data[4*i+1] + greenlpfGainDown_ * outputImages_[0].data[4*i+1];

          // Yellow
          if (outputImages_[1].data[4*i+2] > outputImages_[0].data[4*i+2])
            outputImages_[1].data[4*i+2] = (1-bluelpfGainUp_) * outputImages_[1].data[4*i+2] + bluelpfGainUp_ * outputImages_[0].data[4*i+2];
          else
            outputImages_[1].data[4*i+2] = (1-bluelpfGainDown_) * outputImages_[1].data[4*i+2] + bluelpfGainDown_ * outputImages_[0].data[4*i+2];
        }
    }



    if (generateCombinedImage_)
    {
        if (totalDifference < 40) {
             outputImageCombined.data[4*i] = (int)fmin(outputImages_[1].data[4*i] + imageEdgeDetection.data[i], 255.0);
             outputImageCombined.data[4*i+1] = (int)fmin(outputImages_[1].data[4*i+1]  + imageEdgeDetection.data[i], 255.0);
             outputImageCombined.data[4*i+2] = (int)fmin(outputImages_[1].data[4*i+2]  + imageEdgeDetection.data[i],255.0);
        }
        else {
            outputImageCombined.data[4*i] = outputImages_[1].data[4*i];
            outputImageCombined.data[4*i+1] = outputImages_[1].data[4*i+1];
            outputImageCombined.data[4*i+2] = outputImages_[1].data[4*i+2];
        }
    }

    // Center of Gravity Calculation:
    if (getCOG_) {
      if (totalDifference >= 40) {
        accumulatedTotalDifferenceTimesXAxis += totalDifference * pixelXAxis;
        accumulatedTotalDifferenceTimesYAxis += totalDifference * pixelYAxis;
        accumulatedTotalDifference += totalDifference;
      }
    }

  }

  //! TODO: Add nan check!

  // Center of Gravity Calculation:
  //std::cout << "Center of Gravity X Axis: " << accumulatedTotalDifferenceTimesXAxis / double(accumulatedTotalDifference) << std::endl;
  //std::cout << "Center of Gravity Y Axis: " << accumulatedTotalDifferenceTimesYAxis / double(accumulatedTotalDifference) << std::endl;
  //std::cout << "Accumulated total difference: " << accumulatedTotalDifference << std::endl;


  if (getCOG_) {
    geometry_msgs::Twist cog_values;
    cog_values.linear.x = accumulatedTotalDifferenceTimesXAxis / double(accumulatedTotalDifference);
    cog_values.linear.y = accumulatedTotalDifferenceTimesYAxis / double(accumulatedTotalDifference);
    cog_values.linear.z = accumulatedTotalDifference;
    cogPublisher_.publish(cog_values);

    if (false) {
      if (!isnan(cog_values.linear.x) && !isnan(cog_values.linear.y)){
        std::cout << "got here" << std::endl;
        int iResidual = 4 * round(cog_values.linear.x * outputImage.width);
        int jResidual = 4 * round(cog_values.linear.y * outputImage.height);

        int imageIterator = (jResidual - 1) * outputImage.width + iResidual;

        //outputImage.data[imageIterator] = 255;
        if (imageIterator > 8 && imageIterator < outputImage.width * outputImage.height - 8) {
            outputImage.data[imageIterator-4] = 255;
            outputImage.data[imageIterator-8] = 255;
            outputImage.data[imageIterator+4] = 255;
            outputImage.data[imageIterator+8] = 255;
        }

        //for (unsigned int i = imageIterator - 2; i <= imageIterator + 2 ; ++i) {

        //}

      }
    }
  }




  // Mark the cog:

  // if ()
  // Translate to Pixel Coordinates:




  // Helper image for low pass filtering.

  // TODO: VectorVersion instead of copying entire image..
  //outputImageTemp_ = outputImage;


  lpfTrigger_ = true;

  coloredImagePublisher_.publish(outputImages_[1]);


  // Option to publish the combined image.
  if (generateCombinedImage_) coloredCombinedImagePublisher_.publish(outputImageCombined);


  if (publishMusicTwist_) {
    geometry_msgs::Twist musicTwist;

    if (quadraticCorrelation_) musicTwist.linear.x = max(min((float)totalDifferenceMusicValue * totalDifferenceMusicValue / (gainDivider_ * 1000000000000.0) - minusTerm_, 0.5) ,lowerBound_);
    else musicTwist.linear.x = max(min((float)totalDifferenceMusicValue / 15000000000.0 - 0.07, 0.5) ,0.0024);

    // Pre LPFing
    if (preLPFTrigger_ == true) musicTwist.linear.x = musicTwist.linear.x * (1.0 - preLPFMusicGain_) + oldMusicValue_ * preLPFMusicGain_;
    oldMusicValue_ = musicTwist.linear.x;
    preLPFTrigger_ = true;

    musicTwist.linear.y = max(min(3.0 * musicTwist.linear.x ,0.5) ,0.0);

    MusicValuePublisher_.publish(musicTwist);
  }

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
