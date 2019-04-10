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
  else topic_name = "/camera/rgb/image_color";

  inputImageSubscriber_ = nodeHandle_.subscribe(topic_name, 1, &GravityEffector::inputImageCallback, this);

  outputImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>("/gravity_output_image", 1000);

  cogSubscriber_ = nodeHandle_.subscribe("/motionVisualizerNode/cog_keyvalues", 1, &GravityEffector::cogCallback, this);

  // Trigger for low pass filtering of the cog keyvalues.
  lpfcogTrigger_ = false;
  lpfcogX_ = 0.5;
  lpfcogY_ = 0.5;
  lpfaccumulatedDifference_ = 0;

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
  outputImage.encoding = "bgr8";
  outputImage.width = inputImage.width;
  outputImage.height = inputImage.height;
  outputImage.step = inputImage.step;
  outputImage.is_bigendian = inputImage.is_bigendian;
  outputImage.header = inputImage.header;
  outputImage.data.resize(inputImage.data.size());


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
  std::cout << "Image type " << inputImage.encoding << std::endl;


  if (inputImage.encoding == "bgr8") //std::cout << "Recognized the type" << std::endl;
  {



  // TODO: Consider low pass filtering of COG..

  std::cout << "COG X: " << cogX_ << std::endl;
  std::cout << "COG Y: " << cogY_ << std::endl;
  std::cout << "acc difference: " << accumulatedDifference_ << std::endl;

  double cogX = cogX_;
  double cogY = cogY_;

  // TODO: Colorize the COG!

  int indexHeight = 3 * int(cogY_ * outputImage.height);

  int indexWidthResidual = 3 * round(cogX_ * outputImage.width);

  //std::cout << "1" << std::endl;

  if (!isnan(cogX) && !isnan(cogY) && 0.0 <= cogX <= 1.0 && 0.0 <= cogY <= 1.0){

      for (unsigned int i = 0; i < inputImage.data.size(); i+=3){
          //std::cout << "2" << std::endl;

          //outputImage.data[i] = inputImage.data[i];
          //outputImage.data[i+1] = inputImage.data[i+1];
          //outputImage.data[i+2] = inputImage.data[i+2];

          if (i > indexHeight * outputImage.width && i < (indexHeight+2) * outputImage.width){

              if (i > indexHeight * outputImage.width + indexWidthResidual && i < (indexHeight) * outputImage.width + indexWidthResidual + 4){
                  if (30 < i < outputImage.data.size() - 30){
                    outputImage.data[i] = 255;
                    outputImage.data[i+1] = 255;
                    outputImage.data[i+2] = 255;
                    outputImage.data[i-1] = 255;
                    outputImage.data[i-2] = 255;
                    outputImage.data[i-3] = 255;
                    //outputImage.data[i-4] = 255;
                    //outputImage.data[i-5] = 255;
                    outputImage.data[i+3] = 255;
                    //outputImage.data[i+4] = 255;
                    //outputImage.data[i+5] = 255;
                    //outputImage.data[i+6] = 255;
                  }


              }
          }
          //std::cout << "3" << std::endl;


          //std::cout << "4" << std::endl;



          double xAxisVal = double((int)floor((double)i/3.0) % inputImage.width) / (double)inputImage.width;
          double yAxisVal = double(floor(floor((double)i/3.0) / inputImage.width)+1) / (double)inputImage.height;

          //std::cout << "Vals:  x: " << xAxisVal << " Y: " << yAxisVal << std::endl;

          //std::cout << "5" << std::endl;

          double vecX = cogX - xAxisVal;
          double vecY = cogY - yAxisVal;

          double dist = sqrt(pow(vecX,2)+pow(vecY,2));

          double factor = max(min(0.16 /dist, 0.7),0.0);

          double factorCorr = min(accumulatedDifference_ * factor / 3849156.0, 0.8);

          double xGoal = xAxisVal + factorCorr * vecX;
          double yGoal = yAxisVal + factorCorr * vecY;

          //std::cout << "xgoal: " << xGoal << std::endl;
          //std::cout << "ygoal: " << yGoal << std::endl;
          //std::cout << "xAxisVal: " << xAxisVal << std::endl;
          //std::cout << "yAxisVal: " << yAxisVal << std::endl;
          //std::cout << "cogX: " << cogX << std::endl;
          //std::cout << "cogY: " << cogY << std::endl;


          int indexHeight = int(yGoal * outputImage.height);
          int indexWidthResidual = round(xGoal * outputImage.width);

          //std::cout << "6" << std::endl;
          //std::cout << "indexHeight goeal: " << indexHeight << std::endl;
          //std::cout << "indexWidth goeal: " << indexWidthResidual << std::endl;

          int indexTotal = 3 * indexHeight * outputImage.width + 3 * indexWidthResidual;

          if (indexTotal <= outputImage.data.size()-3 && indexTotal > 0) {
            //outputImage.data[indexTotal-1] = inputImage.data[i-1];
            outputImage.data[indexTotal] = inputImage.data[i];
            outputImage.data[indexTotal+1] = inputImage.data[i+1];
            outputImage.data[indexTotal+2] = inputImage.data[i+2];
          }
          // Do the gravity Effecting thingy here!
        }
     }
    outputImagePublisher_.publish(outputImage);
  }




}

void GravityEffector::cogCallback(geometry_msgs::Twist cogKeyvalues)
{
    //std::cout << "cogKeys: x " << cogKeyvalues.linear.x << std::endl;
    cogX_ = cogKeyvalues.linear.x;
    cogY_ = cogKeyvalues.linear.y;
    accumulatedDifference_ = cogKeyvalues.linear.z;

    double lpffactor = 0.87;

    if (lpfcogTrigger_ == true) {
      lpfcogX_ = lpffactor * lpfcogX_ + (1 - lpffactor) * cogX_;
      lpfcogY_ = lpffactor * lpfcogY_ + (1 - lpffactor) * cogY_;
      lpfcogX_ = lpffactor * lpfaccumulatedDifference_ + (1 - lpffactor) * accumulatedDifference_;
    }

    if (lpfcogTrigger_ == false) {
        lpfcogX_ = cogX_;
        lpfcogY_ = cogY_;
        lpfaccumulatedDifference_ = accumulatedDifference_;
        lpfcogTrigger_ = true;
    }
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
