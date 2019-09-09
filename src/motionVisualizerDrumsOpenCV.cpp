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
#include <cv_bridge/cv_bridge.h>
//#include "effects/builtin/filtereffect.h"
//#include <image_transport/image_transport.h>


#include "simple_kinect_motion_visualizer/motionVisualizerDrumsOpenCV.hpp"

using namespace std;
//using namespace grid_map;
//using namespace ros;
//using namespace tf;
//using namespace pcl;
//using namespace kindr;
//using namespace kindr_ros;


MotionVisualizerDrumsOpenCV::MotionVisualizerDrumsOpenCV(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  ROS_INFO("Motion visualization node for Drum Machine started.");

  std::string topic_name;
  bool depth_image = false;
  bool mono_image = true;
  if (depth_image) topic_name = "/camera/depth/image";
  else if (mono_image) topic_name = "/camera/rgb/image_mono";
  else topic_name = "/edge_detection/image";


  // Image transport
  //image_transport::ImageTransport it(nodeHandle_);

  //inputImageSubscriber_ = it.subscribe(topic_name, 1, &MotionVisualizerDrumsOpenCV::edgeDetectionImageCallback, this);



  edgeDetectionImageSubscriber_ = nodeHandle_.subscribe(topic_name, 1, &MotionVisualizerDrumsOpenCV::edgeDetectionImageCallback, this);

  // Helper subscriber for frequency test, may be obsolete soon.
  //cameraInfoSubscriber_ = nodeHandle_.subscribe("/camera/rgb/camera_info", 1, &MotionVisualizerDrums::cameraInfoCallback, this);

  coloredImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>("/colored_image2", 1);
  //coloredCombinedImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>("/colored_image_combined2", 5);

  MusicValuePublisher_ = nodeHandle_.advertise<geometry_msgs::Pose2D>("music_values", 1);
  DifferentialMusicValuePublisher_ = nodeHandle_.advertise<geometry_msgs::Pose2D>("music_values_differential", 1);

  MusicValuePublisherForLED_ = nodeHandle_.advertise<geometry_msgs::Pose2D>("music_values_for_led", 1);

  //std::cout << "before 1" << std::endl;
  drumMessagePublisher_ = nodeHandle_.advertise<geometry_msgs::Pose2D>("/motion_visualizer/drum_triggers", 1);
  //std::cout << "after 1" << std::endl;

  //cogPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("cog_keyvalues", 5);

  // Trigger for low pass filter.
  lpfTrigger_ = false;
  preLPFTrigger_ = false;

  // Bool to chose if generating combined image
  generateCombinedImage_ = false;

  // Bool to chose if calculating cog of motion
  getCOG_ = false;

  // Bool to chose if publishing music values
  publishMusicTwist_ = true;


  alternator_ = false;


  // Trigger for dfferential Derivative calculation.
  differentiationtrigger_ = false;

  // Trigger for simple differential Derivative calculation.
  diffTrigger_ = false;
  diffLPFTrigger_ = false;


  // Initialize basic motor velocity.
  basicMotorVelocityX_ = 0;
  basicMotorVelocityY_ = 0;
  basicMotorVelocityTHETA_ = 0;

  // Initialize the drum activation array to 0.

  for (unsigned int i = 0; i < 144; ++i) {
      drumActivationInFieldsArray_[i] = false;
      drumActivationInFieldsArrayOld_[i] = false;
      colorizationIntensityArray_[i] = 0;
  }
  //memset(drumActivationInFieldsArray_, 0, sizeof(drumActivationInFieldsArray_));
  //memset(drumActivationInFieldsArrayOld_, 0, sizeof(drumActivationInFieldsArrayOld_));
  //memset(colorizationIntensityArray_, 0, sizeof(colorizationIntensityArray_));

  //drumActivationInFieldsArray_[] = {};
  //drumActivationInFieldsArrayOld_[] = {};

  // Initialize the timers.
  newDrumTriggerTime_ = ros::Time::now().toSec();
  oldDrumTriggerTime_ = ros::Time::now().toSec();

  // Chose if applying lpfing for image.
  applyLPF_ = false;
}


void MotionVisualizerDrumsOpenCV::edgeDetectionImageCallback(
    const sensor_msgs::ImageConstPtr& imageEdgeDetection)
{
  //ROS_INFO("Image encoding: %s", imageEdgeDetection.encoding.c_str());
  //ROS_INFO("200 entry data: %d", imageEdgeDetection.data[1000]);

  //sensor_msgs::Image outputImage;
  //outputImage.encoding = "rgba8";
  //outputImage.width = imageEdgeDetection.width;
  //outputImage.height = imageEdgeDetection.height;
  //outputImage.step = imageEdgeDetection.step * 4;
  //outputImage.is_bigendian = imageEdgeDetection.is_bigendian;
  //outputImage.header = imageEdgeDetection.header;
  //outputImage.data.resize(4*imageEdgeDetection.data.size());

  sensor_msgs::Image outputImage;
  outputImage.encoding = "rgba8";
  outputImage.width = imageEdgeDetection->width;
  outputImage.height = imageEdgeDetection->height;
  outputImage.step = imageEdgeDetection->step * 4;
  outputImage.is_bigendian = imageEdgeDetection->is_bigendian;
  outputImage.header = imageEdgeDetection->header;
  outputImage.data.resize(4*imageEdgeDetection->data.size());



  //std::cout << "width: " << imageEdgeDetection->width << "height: " << imageEdgeDetection->height << "Encoding: " << imageEdgeDetection->encoding << std::endl;

  //sensor_msgs::Image outputImageCombined;
  //outputImageCombined.encoding = "rgba8";
  //outputImageCombined.width = imageEdgeDetection.width;
  //outputImageCombined.height = imageEdgeDetection.height;
  //outputImageCombined.step = imageEdgeDetection.step * 4;
  //outputImageCombined.is_bigendian = imageEdgeDetection.is_bigendian;
  //outputImageCombined.header = imageEdgeDetection.header;
  //outputImageCombined.data.resize(4*imageEdgeDetection.data.size());


  // Set number of considered images for blurring
  if (edgeDetectionImageHistory_.size() > 1) edgeDetectionImageHistory_.erase(edgeDetectionImageHistory_.begin()); // Hacked a 2 in here, history sizes will have no effect anymore
  edgeDetectionImageHistory_.push_back(*imageEdgeDetection);

  if (outputImages_.size() > 1) outputImages_.erase(outputImages_.begin()); // Hacked a 2 in here, history sizes will have no effect anymore
  outputImages_.push_back(outputImage);

  //std::cout << "edgeDetectionImageHistory__size: " << edgeDetectionImageHistory_.size() << std::endl;



  // TEST for music value consistency:

  //ROS_INFO("This is the motor Value of THETA (right): %d", basicMotorVelocityTHETA_);



  int totalDifferenceMusicValue = 0;

  // TODO: do stuff with this.
  int totalDifferenceMusicValueLeft = 0;
  int totalDifferenceMusicValueRight = 0;

  // Fot calculation of pixel center of gravity:
  //int totalDifferenceTimesHorizontalPixels;
  //int totalDOfferenceTimesVertialPixels;

  //std::cout << "Size of the Edge detection image: " << imageEdgeDetection.data.size() << std::endl;

  //imageEdgeDetection.header.

  // For center of gravity calculation.

  // TODO: Check if these are deprecated.
  int accumulatedTotalDifference = 0;
  double accumulatedTotalDifferenceTimesXAxis = 0.0;
  double accumulatedTotalDifferenceTimesYAxis = 0.0;


  // Vector of Difference Values for each field.
  int differenceValueInFieldsArray[144] = {};


  //std::cout << "BIS HIER HIN BIN ICH NOCH EASY GEKOMMEN" << std::endl;

  for (unsigned int i = 0; i < imageEdgeDetection->data.size(); ++i){
    //outputImage.data[4*i] = imageEdgeDetection.data[i];
    // Get location within image: x = 0: left border, x = 1 reight border, y = 0: top, y = 0: bottom

    double pixelXAxis = double(i % imageEdgeDetection->width) / (double)imageEdgeDetection->width;
    //double pixelYAxis = double(floor(i / imageEdgeDetection.width)+1) / (double)imageEdgeDetection.height;
    double pixelYAxis = double(floor((double)i / (double)imageEdgeDetection->width)) / (double)imageEdgeDetection->height;


    //std::cout << "YAxis: " << pixelYAxis << std::endl;


    // Version: Tiny fields at left hand corner.


    int fieldXTinyLeft, fieldYTinyLeft, fieldXTinyRight, fieldYTinyRight;
    int fieldNumberTiny;



    // Numbers to chose (DynRec!)
    double widthSideStripes = 1.0/3.0;
    int noHorizontalFieldsInSideStripes = 6;
    int noVerticalFieldsInSideStripes = 12;
    int fieldNumber = 0;
    bool useTinyAdjustable = true;
    bool noField = true;


    //std::cout << "BIS HIER HIN BIN ICH NOCH EASY GEKOMMEN 222222" << std::endl;

    if (useTinyAdjustable)
    {
      // LEFT THIRD:
      if (pixelXAxis <= widthSideStripes)
      {
        fieldXTinyLeft = floor(pixelXAxis * noHorizontalFieldsInSideStripes * (1.0/widthSideStripes)) + 1;
        fieldYTinyLeft = floor(pixelYAxis * noVerticalFieldsInSideStripes) + 1;
        fieldNumber = fieldXTinyLeft + (fieldYTinyLeft - 1) * noHorizontalFieldsInSideStripes;
        noField = false;
      }

      // RIGHT THIRD:
      if (pixelXAxis > 1 - widthSideStripes)
      {
        fieldXTinyRight = floor((pixelXAxis - (1 - widthSideStripes)) * noHorizontalFieldsInSideStripes * (1.0/widthSideStripes)) + 1;
        fieldYTinyRight = floor(pixelYAxis * noVerticalFieldsInSideStripes) + 1;
        //std::cout << "FieldX: " << fieldXTinyRight << " fieldy " << fieldYTinyRight << std::endl;
        fieldNumber = fieldXTinyRight + (fieldYTinyRight - 1) * noHorizontalFieldsInSideStripes + noHorizontalFieldsInSideStripes * noVerticalFieldsInSideStripes;
        noField = false;
      }
    }

    else {
      // 6 Columns
      int fieldX = floor(pixelXAxis * 6.0) + 1;
      // 3 Rows
      int fieldY = floor(pixelYAxis * 3.0) + 1;

      fieldNumber = (fieldY - 1) * 6 + fieldX;
      noField = false;
    }

    //std::cout << "BIS HIER HIN BIN ICH NOCH EASY GEKOMMEN 33333" << std::endl;

    //std::cout << "This is the field Number: " << fieldNumber << std::endl;



    // 18 Fields, depending on Pixel position
    // Formula: (fieldY - 1) * 6 + fieldX




    int totalDifference = 0;
    bool fast = false;


    std::cout << "Edge Detection image history size: " << edgeDetectionImageHistory_.size() << std::endl;

    for(unsigned int j = 0; j < edgeDetectionImageHistory_.size()-1; ++j){
      totalDifference += fabs(edgeDetectionImageHistory_[j].data[i] - edgeDetectionImageHistory_[j+1].data[i]);
    }

    //std::cout << "BIS HIER HIN BIN ICH NOCH EASY GEKOMMEN 55555" << std::endl;

    int musicIntensityTreshold = 50;
    if (totalDifference > musicIntensityTreshold) {
        totalDifferenceMusicValue += totalDifference;
        if (pixelXAxis <= 0.5) totalDifferenceMusicValueLeft += totalDifference;
        if (pixelXAxis >= 0.5) totalDifferenceMusicValueRight += totalDifference;
        if (!noField) differenceValueInFieldsArray[fieldNumber-1] += totalDifference;
    }

    // Low pass filtering step.
    if (lpfTrigger_) {
        if (outputImages_.size() > 1) {
            if (false && totalDifference <= redIntensityThreshold_ && totalDifference <= blueIntensityThreshold_ && totalDifference <= greenIntensityThreshold_)
            {
                outputImages_[1].data[4*i] = 0;
                outputImages_[1].data[4*i+1] = 0;
                outputImages_[1].data[4*i+2] = 0;
            }
            else
            {
              if (!fast){
                outputImages_[1].data[4*i] = fmin(255, redGain_ * totalDifference);
                //if (totalDifference <= redIntensityThreshold_) outputImages_[1].data[4*i] = 0;
                if (applyLPF_) {

                if (outputImages_[1].data[4*i] <= lpfRedIntensityThreshold_ && outputImages_[0].data[4*i] <= lpfRedIntensityThreshold_) outputImages_[1].data[4*i] = 0;
                else {
                    if (fabs(outputImages_[1].data[4*i] - outputImages_[0].data[4*i]) <= lpfRedIntensityThreshold_) outputImages_[1].data[4*i] = outputImages_[0].data[4*i];
                    else {
                        // Red
                        if (outputImages_[1].data[4*i] > outputImages_[0].data[4*i])
                          outputImages_[1].data[4*i] = (1-redlpfGainUp_) * outputImages_[1].data[4*i] + redlpfGainUp_ * outputImages_[0].data[4*i];
                        else
                          outputImages_[1].data[4*i] = (1-redlpfGainDown_) * outputImages_[1].data[4*i] + redlpfGainDown_ * outputImages_[0].data[4*i];
                    }
                }
                }

                outputImages_[1].data[4*i+1] = fmin(255, greenGain_ * totalDifference);
                //if (totalDifference <= greenIntensityThreshold_) outputImages_[1].data[4*i+1] = 0;

                if (applyLPF_) {
                if (outputImages_[1].data[4*i+1] <= lpfGreenIntensityThreshold_ && outputImages_[0].data[4*i+1] <= lpfGreenIntensityThreshold_) outputImages_[1].data[4*i+1] = 0;
                else {
                    if (fabs(outputImages_[1].data[4*i+1] - outputImages_[0].data[4*i+1]) <= lpfGreenIntensityThreshold_) outputImages_[1].data[4*i+1] = outputImages_[0].data[4*i+1];
                    else {
                        // Green
                        if (outputImages_[1].data[4*i+1] > outputImages_[0].data[4*i+1])
                          outputImages_[1].data[4*i+1] = (1-greenlpfGainUp_) * outputImages_[1].data[4*i+1] + greenlpfGainUp_ * outputImages_[0].data[4*i+1];
                        else
                          outputImages_[1].data[4*i+1] = (1-greenlpfGainDown_) * outputImages_[1].data[4*i+1] + greenlpfGainDown_ * outputImages_[0].data[4*i+1];
                    }
                }
                }

                outputImages_[1].data[4*i+2] = fmin(255, blueGain_ * totalDifference);
                //if (totalDifference <= blueIntensityThreshold_) outputImages_[1].data[4*i+2] = 0;

                if (applyLPF_) {
                if (outputImages_[1].data[4*i+2] <= lpfBlueIntensityThreshold_ && outputImages_[0].data[4*i+2] <= lpfBlueIntensityThreshold_) outputImages_[1].data[4*i+2] = 0;
                else {
                    if (fabs(outputImages_[1].data[4*i+2] - outputImages_[0].data[4*i+2]) <= lpfBlueIntensityThreshold_) outputImages_[1].data[4*i+2] = outputImages_[0].data[4*i+2];
                    else {
                        // Blue
                        if (outputImages_[1].data[4*i+2] > outputImages_[0].data[4*i+2])
                          outputImages_[1].data[4*i+2] = (1-bluelpfGainUp_) * outputImages_[1].data[4*i+2] + bluelpfGainUp_ * outputImages_[0].data[4*i+2];
                        else
                          outputImages_[1].data[4*i+2] = (1-bluelpfGainDown_) * outputImages_[1].data[4*i+2] + bluelpfGainDown_ * outputImages_[0].data[4*i+2];
                    }
                }
                }
              }

            }
        }
        //std::cout << "BIS HIER HIN BIN ICH NOCH EASY GEKOMMEN 55555" << std::endl;


        // TEST for fully coloring according to detected motion
        if (!noField) {
          if ((fieldNumber-1) % 14 == 0) outputImages_[1].data[4*i+1] = max(min(colorizationIntensityArray_[fieldNumber-1], 255), 0);
          else if ((fieldNumber-1) % 13 == 0) outputImages_[1].data[4*i] = max(min(colorizationIntensityArray_[fieldNumber-1], 255), 0);
          else outputImages_[1].data[4*i+2] = max(min(colorizationIntensityArray_[fieldNumber-1], 255), 0);
        }
        //if (fieldNumber % 12 == 0) outputImages_[1].data[4*i+1] = max(min(colorizationIntensityArray_[fieldNumber-1], 255), 0);







    }




    //if (generateCombinedImage_)
    //{
    //    if (totalDifference < 40) {
    //         outputImageCombined.data[4*i] = (int)fmin(outputImages_[1].data[4*i] + imageEdgeDetection.data[i], 255.0);
    //         outputImageCombined.data[4*i+1] = (int)fmin(outputImages_[1].data[4*i+1]  + imageEdgeDetection.data[i], 255.0);
    //         outputImageCombined.data[4*i+2] = (int)fmin(outputImages_[1].data[4*i+2]  + imageEdgeDetection.data[i],255.0);
    //    }
    //    else {
    //        outputImageCombined.data[4*i] = outputImages_[1].data[4*i];
    //        outputImageCombined.data[4*i+1] = outputImages_[1].data[4*i+1];
    //        outputImageCombined.data[4*i+2] = outputImages_[1].data[4*i+2];
    //    }
    //}

    // Center of Gravity Calculation:
    //if (getCOG_) {
    //  if (totalDifference >= 40) {
    //    accumulatedTotalDifferenceTimesXAxis += totalDifference * pixelXAxis;
    //    accumulatedTotalDifferenceTimesYAxis += totalDifference * pixelYAxis;
    //    accumulatedTotalDifference += totalDifference;
    //  }
    //}

  }
  //std::cout << "BIS HIN BIN ICH NOCH EASY GEKOMMEN 2" << std::endl;
  // Fade out coloring intensity.



  //! TODO: Add nan check!

  // Center of Gravity Calculation:
  //std::cout << "Center of Gravity X Axis: " << accumulatedTotalDifferenceTimesXAxis / double(accumulatedTotalDifference) << std::endl;
  //std::cout << "Center of Gravity Y Axis: " << accumulatedTotalDifferenceTimesYAxis / double(accumulatedTotalDifference) << std::endl;
  //std::cout << "Accumulated total difference: " << accumulatedTotalDifference << std::endl;


  //if (getCOG_) {
  //  geometry_msgs::Twist cog_values;
  //  cog_values.linear.x = accumulatedTotalDifferenceTimesXAxis / double(accumulatedTotalDifference);
  //  cog_values.linear.y = accumulatedTotalDifferenceTimesYAxis / double(accumulatedTotalDifference);
  //  cog_values.linear.z = accumulatedTotalDifference;
  //  cogPublisher_.publish(cog_values);

  //  if (false) {
  //    if (!isnan(cog_values.linear.x) && !isnan(cog_values.linear.y)){
  //      //std::cout << "got here" << std::endl;
  //      int iResidual = 4 * round(cog_values.linear.x * outputImage.width);
  //      int jResidual = 4 * round(cog_values.linear.y * outputImage.height);

  //      int imageIterator = (jResidual - 1) * outputImage.width + iResidual;

        //outputImage.data[imageIterator] = 255;
  //      if (imageIterator > 8 && imageIterator < outputImage.width * outputImage.height - 8) {
  //          outputImage.data[imageIterator-4] = 255;
  //          outputImage.data[imageIterator-8] = 255;
  //          outputImage.data[imageIterator+4] = 255;
   //         outputImage.data[imageIterator+8] = 255;
   //     }

        //for (unsigned int i = imageIterator - 2; i <= imageIterator + 2 ; ++i) {

        //}

     // }
  //  }
  //}




  // Mark the cog:

  // if ()
  // Translate to Pixel Coordinates:




  // Helper image for low pass filtering.

  // TODO: VectorVersion instead of copying entire image..
  //outputImageTemp_ = outputImage;


  lpfTrigger_ = true;



  //std::cout << "BIS HIER HIN BIN ICH NOCH EASY GEKOMMEN 3 size of output images list: " << outputImages_.size() << std::endl;
  if (outputImages_.size() > 1) {
    //std::cout << "before 3" << std::endl;
    coloredImagePublisher_.publish(outputImages_[1]);
    //std::cout << "after 3" << std::endl;
  }

  //std::cout << "BIS HIER HIN BIN ICH NOCH EASY GEKOMMEN 4" << std::endl;


  // Option to publish the combined image.
  //if (generateCombinedImage_) coloredCombinedImagePublisher_.publish(outputImageCombined);

  if (publishMusicTwist_) {
    // geometry_msgs::Twist musicTwist; // Arduino Efficiency Hack

    double musicValueTotal;

    //if (quadraticCorrelation_) musicTwist.linear.x = max(min((float)totalDifferenceMusicValue * totalDifferenceMusicValue / (gainDivider_ * 1000000000000.0) - minusTerm_, 0.5) ,lowerBound_);
    //else musicTwist.linear.x = max(min((float)totalDifferenceMusicValue / 15000000000.0 - 0.07, 0.5) ,0.0024);
    totalDifferenceMusicValue = abs(totalDifferenceMusicValue);
    totalDifferenceMusicValueLeft = abs(totalDifferenceMusicValueLeft);
    totalDifferenceMusicValueRight = abs(totalDifferenceMusicValueRight);



    // TODO: Removed a factor of 100 here:
    if (quadraticCorrelation_) musicValueTotal = max(min((float)totalDifferenceMusicValue * totalDifferenceMusicValue / (gainDivider_ * 10000000000.0) - minusTerm_, 0.5) ,lowerBound_);
    else musicValueTotal = max(min((float)totalDifferenceMusicValue / 15000000000.0 - 0.07, 0.5) ,lowerBound_);


    //std::cout << "musicValueTotal 1: " << musicValueTotal << std::endl;
    //std::cout << "DifferenceMusicValue 1 squared: " << totalDifferenceMusicValue * totalDifferenceMusicValue << std::endl;
    //std::cout << "Music Val Tot: " << totalDifferenceMusicValue << " Left: " << totalDifferenceMusicValueLeft << " Right: " << totalDifferenceMusicValueRight << std::endl;

    bool leftRight = true;
    double musicLeft, musicRight;
    totalDifferenceMusicValueLeft *= 2.0;
    totalDifferenceMusicValueRight *= 2.0;
    if (leftRight) {
      if (quadraticCorrelation_) musicLeft = max(min((float)totalDifferenceMusicValueLeft * totalDifferenceMusicValueLeft / (gainDivider_ * 10000000000.0) - minusTerm_, 0.5) ,lowerBound_);
      else musicLeft = max(min((float)totalDifferenceMusicValueLeft / 15000000000.0 - 0.07, 0.5) ,lowerBound_);

      if (quadraticCorrelation_) musicRight = max(min((float)totalDifferenceMusicValueRight * totalDifferenceMusicValueRight / (gainDivider_ * 10000000000.0) - minusTerm_, 0.5) ,lowerBound_);
      else musicRight = max(min((float)totalDifferenceMusicValueRight / 15000000000.0 - 0.07, 0.5) ,lowerBound_);
    }


    // Get Music value for each Field
    double musicTriggerValuesInFieldsArray[144] = {};
    // Take absolute values (just for security).
    for (unsigned int k = 0; k < 144; ++k){
        differenceValueInFieldsArray[k] = abs(144.0 * differenceValueInFieldsArray[k]);
        if (quadraticCorrelation_) musicTriggerValuesInFieldsArray[k] = max(min((float)differenceValueInFieldsArray[k] * differenceValueInFieldsArray[k] / (gainDivider_ * 10000000000.0) - minusTerm_, 0.5) ,lowerBound_);
        else musicTriggerValuesInFieldsArray[k] = max(min((float)differenceValueInFieldsArray[k] / 15000000000.0 - 0.07, 0.5) ,lowerBound_);
        //differenceValueInFieldsArray[k] = abs(18.0 * differenceValueInFieldsArray[k]);
    }

    /// Differential approximation of derivative

    // Check Coding rules for this
    double musicTwistDifferential = 0.0, musicLeftDifferential = 0.0, musicRightDifferential = 0.0;

    if(differentiationtrigger_) {
      if (differentiationtrigger_) {
          musicTwistDifferential = fabs(musicValueTotal - musicTwistOldNoFilter_);
          musicLeftDifferential = fabs(musicLeft - musicLeftOldNoFilter_);
          musicRightDifferential = fabs(musicRight - musicRightOldNoFilter_);
      }
    }

    musicTwistOldNoFilter_ = musicValueTotal;
    musicLeftOldNoFilter_ = musicLeft;
    musicTwistOldNoFilter_ = musicRight;

    if (differentiationtrigger_ == false) differentiationtrigger_ = true;

    //std::cout << "musicValueTotal: " << musicValueTotal << std::endl;

    // Pre LPFing
    if (preLPFTrigger_ == true) musicValueTotal = musicValueTotal * (1.0 - lpfGainBasicX_) + oldMusicValue_ * lpfGainBasicX_;
    oldMusicValue_ = musicValueTotal;

    // PreLPFing of differential version
    //if (preLPFTrigger_ == true) musicTwistDifferential = musicTwistDifferential * (1.0 - preLPFMusicGainDifferential_) + musicTwistOldFiltering_ * preLPFMusicGainDifferential_;
    //musicTwistOldFiltering_ = musicTwistDifferential;



    // Pre LPF leftright
    if (leftRight) {
        if (preLPFTrigger_ == true) musicLeft = musicLeft * (1.0 - lpfGainBasicY_) + oldMusicValueLeft_ * lpfGainBasicY_;
        oldMusicValueLeft_ = musicLeft;
        if (preLPFTrigger_ == true) musicRight = musicRight * (1.0 - lpfGainBasicTHETA_) + oldMusicValueRight_ * lpfGainBasicTHETA_;
        oldMusicValueRight_ = musicRight;

        //Differential Version
        //if (preLPFTrigger_ == true) musicLeftDifferential = musicLeftDifferential * (1.0 - preLPFMusicGainDifferential_) + musicLeftOldFiltering_ * preLPFMusicGainDifferential_;
        //musicLeftOldFiltering_ = musicLeftDifferential;
        //if (preLPFTrigger_ == true) musicRightDifferential = musicRightDifferential * (1.0 - preLPFMusicGainDifferential_) + musicRightOldFiltering_ * preLPFMusicGainDifferential_;
        //musicRightOldFiltering_ = musicRightDifferential;

    }

    preLPFTrigger_ = true;

    //musicTwist.linear.y = max(min(3.0 * musicTwist.x, 0.5), 0.0);

    geometry_msgs::Pose2D musicTwist;

    //std::cout << "BIS HIER HIN BIN ICH NOCH EASY GEKOMMEN 5" << std::endl;

    // Basic motor velocities.

    //int basicMotorVelocityInFieldsArray[18] = {};

    int activationThreshold = 95; // To be dynamically reconfigured!!! TODO

    for (unsigned int k = 0; k < 144; ++k) {
        // Attention hacked a factor 1/4.
        basicMotorVelocityInFieldsArray_[k] = (int)fmax(fmin(0 + sensitivityBasicTHETA_ * (musicTriggerValuesInFieldsArray[k]/3.0 - lowerBound_), maxVelocityBasicTHETA_), 0.0);
        //std::cout << "Drum activation: " <<
        if (basicMotorVelocityInFieldsArray_[k] >= activationThreshold && drumActivationInFieldsArray_[k] == false) {
            drumActivationInFieldsArray_[k] = true;
            //std::cout << "Ever got here? is it a coloring issue? " << basicMotorVelocityInFieldsArray_[k] << std::endl;
        }
    }

    // TODO: Eventually just one loop.

    //std::cout << "BIS HIER HIN BIN ICH NOCH EASY GEKOMMEN 6" << std::endl;


    // Field activation and release.
    for (unsigned int k = 0; k < 144; ++k) {
      //std::cout << "Ever got here? is it a coloring issue? " << basicMotorVelocityInFieldsArray_[k] << std::endl;
      // Case Drum just activated.
      //std::cout << "BIS HIER HIN BIN ICH NOCH EASY GEKOMMEN 8: " << k << std::endl;
      if (drumActivationInFieldsArray_[k] == true && drumActivationInFieldsArrayOld_[k] == false){
          //std::cout << "Ever got here? is it a coloring issue? " << std::endl;
          colorizationIntensityArray_[k] = 255;
          // Here send a message to the Drum Machine.
          geometry_msgs::Pose2D drumMsg;


          // Three layers: x -> Ornaments (live), y -> Kicks (On 1 and 3 of Metronome vel.), z -> Snares (On 2 and 4 of Metronome Vel)
          if (k % 14 == 0) {
              drumMsg.x = 0;
              drumMsg.theta = 1.0;
          }
          else if (k % 13 == 0) {
              drumMsg.x = 2;
              drumMsg.theta = 0.9;
          }
          else {
              drumMsg.x = 12;
              drumMsg.theta = 0.4;

          }
          drumMsg.y = 0.0;


          //std::cout << "before 2" << std::endl;
          drumMessagePublisher_.publish(drumMsg);
          //std::cout << "after 2" << std::endl;
      }
      // Decrease if it was activated before.
      if (drumActivationInFieldsArray_[k] == true && drumActivationInFieldsArrayOld_[k] == true) colorizationIntensityArray_[k] -= 40;
      if (colorizationIntensityArray_[k] <= 0) {
          colorizationIntensityArray_[k] = 0; // RELEASE THE BLOCKING OF THE FIELD
          drumActivationInFieldsArray_[k] = false;
      }

      //if (drumActivationInFieldsArray_[k] == false)

      // Store previous data for next round.
      drumActivationInFieldsArrayOld_[k] = drumActivationInFieldsArray_[k];

    }

   // std::cout << "BIS HIER HIN BIN ICH NOCH EASY GEKOMMEN 7" << std::endl;

    basicMotorVelocityX_ = (int)fmax(fmin(0 + sensitivityBasicX_ * (musicValueTotal - lowerBound_), maxVelocityBasicX_), 0.0);
    basicMotorVelocityY_ = (int)fmax(fmin(0 + sensitivityBasicY_ * (musicLeft - lowerBound_), maxVelocityBasicY_), 0.0);
    basicMotorVelocityTHETA_ = (int)fmax(fmin(0 + sensitivityBasicTHETA_ * (musicRight - lowerBound_), maxVelocityBasicTHETA_), 0.0);

    musicTwist.x = basicMotorVelocityX_;
    musicTwist.y = basicMotorVelocityY_;
    musicTwist.theta = basicMotorVelocityTHETA_;



    //std::cout << "musicTwist just before publishing: " << musicTwist.x << " musicleft: " << musicTwist.y << " musicRight: " << musicTwist.theta << std::endl;
    newDrumTriggerTime_ = ros::Time::now().toSec();
    if (newDrumTriggerTime_ - oldDrumTriggerTime_ > 0.4 && false)
    {
        geometry_msgs::Pose2D drumTriggerMsg;
        drumTriggerMsg.y = 1.0;
        drumMessagePublisher_.publish(drumTriggerMsg);
        oldDrumTriggerTime_ = newDrumTriggerTime_;
    }

    //std::cout << "BIS HIER HIN BIN ICH NOCH EASY GEKOMMEN 10000" << std::endl;

    // TODO: consider using output values for differential calculation..

    // Ouput Values directly.

    // Avoid saturation of basic motor speeds.
    int motorBasicVelocityX = (int)max(0 + sensitivityBasicX_ * (musicValueTotal - lowerBound_), 0.0);
    int motorBasicVelocityY = (int)max(0 + sensitivityBasicY_ * (musicLeft - lowerBound_), 0.0);
    int motorBasicVelocityTHETA = (int)max(0 + sensitivityBasicTHETA_ * (musicRight - lowerBound_), 0.0);

    // Message to publish.
    geometry_msgs::Pose2D musicSimplerDifferential;

    bool simpleDiffCalculation = true;
    if (simpleDiffCalculation)
    {
      double diffX;
      double diffY;
      double diffTHETA;


      // Diff Loop:
      if (diffTrigger_ == true)
      {

        // TODO: Think about thresholding the diff.
        // Update START
        diffX = fabs((motorBasicVelocityX - XOldForDiff_) * sensitivityDiffX_);
        diffY = fabs((motorBasicVelocityY - YOldForDiff_) * sensitivityDiffY_);
        diffTHETA = fabs((motorBasicVelocityTHETA - THETAOldForDiff_) * sensitivityDiffTHETA_);

        //ROS_INFO("diffX: %f diffY: %f diffTHETA: %f", diffX, diffY, diffTHETA);

        // Diff LPF Loop:
        if (diffLPFTrigger_ == true)
        {
          // TODO: single steps to check outputs..
          musicSimplerDifferential.x = (int)fmax(fmin(lpfGainDiffX_ * XOldDiffForLPF_ + (1.0 - lpfGainDiffX_) * diffX, maxVelocityDiffX_), 0.0);
          musicSimplerDifferential.y = (int)fmax(fmin(lpfGainDiffY_ * YOldDiffForLPF_ + (1.0 - lpfGainDiffY_) * diffY, maxVelocityDiffY_), 0.0);
          musicSimplerDifferential.theta = (int)fmax(fmin(lpfGainDiffTHETA_ * THETAOldDiffForLPF_ + (1.0 - lpfGainDiffTHETA_) * diffTHETA, maxVelocityDiffTHETA_), 0.0);
          //ROS_WARN("diffvelx: %f, diffvely: %f, diffveltheta: %f", musicSimplerDifferential.x, musicSimplerDifferential.y, musicSimplerDifferential.theta);
          //ROS_WARN("lpfGainDiffX: %f, lpfGainDiffY: %f, lpfGainDiffTHETA: %f", lpfGainDiffX_, lpfGainDiffY_, lpfGainDiffTHETA_);
          //ROS_WARN("XOLDDIFFFORLPF minus diffX: %f,, XOldDiffForLPF: %f, diffX: %f", XOldDiffForLPF_ - diffX, XOldDiffForLPF_ ,diffX);

          XOldDiffForLPF_ = musicSimplerDifferential.x;
          YOldDiffForLPF_ = musicSimplerDifferential.y;
          THETAOldDiffForLPF_ = musicSimplerDifferential.theta;
        }

        if (!diffLPFTrigger_) {
        // Assign Old LPF Diff values
        XOldDiffForLPF_ = 0.0;
        YOldDiffForLPF_ = 0.0;
        THETAOldDiffForLPF_ = 0.0;
        }


        diffLPFTrigger_ = true;
      }

      // Assign Old values
      XOldForDiff_ = motorBasicVelocityX;
      YOldForDiff_ = motorBasicVelocityY;
      THETAOldForDiff_ = motorBasicVelocityTHETA;


      diffTrigger_ = true;
      // Update END

    }
    
    bool oldDifferentialCalculation = false;
    if (oldDifferentialCalculation)
    {
      geometry_msgs::Pose2D musicDifferential;
      // TODO: Second Motor mulstiplier and second message publisher for Derivative sensitive motors.
      musicDifferential.x = (int)max(min(0 + motorMultiplierDifferential_ * ((0.2 * musicValueTotal + 0.8 * musicTwistDifferential) - lowerBound_), 800.0), 0.0);
      musicDifferential.y = (int)max(min(0 + motorMultiplierDifferential_ * ((0.2 * musicLeft + 0.8 * musicLeftDifferential) - lowerBound_), 800.0), 0.0);
      musicDifferential.theta = (int)max(min(0 + motorMultiplierDifferential_ * ((0.2 * musicRight + 0.8 * musicRightDifferential) - lowerBound_), 800.0), 0.0);
      DifferentialMusicValuePublisher_.publish(musicDifferential);
    }


    // Randomize and Publish:
    //auto motorSpeedTuple = RandomSwitch(musicTwist, musicSimplerDifferential);
    //geometry_msgs::Pose2D musicTwistRandomized = std::get<0>(motorSpeedTuple);
    //geometry_msgs::Pose2D musicSimplerDifferentialRandomized = std::get<1>(motorSpeedTuple);

    // Publish.
    if (simpleDiffCalculation && diffTrigger_ && diffLPFTrigger_) DifferentialMusicValuePublisher_.publish(musicSimplerDifferential);
    //std::cout << "BIS HIER HIN BIN ICH NOCH EASY GEKOMMEN 10001" << std::endl;
    //MusicValuePublisher_.publish(musicTwist);
  }

  //std::cout << "BIS HIER HIN BIN ICH NOCH EASY GEKOMMEN 10002" << std::endl;








}


//std::tuple<geometry_msgs::Pose2D, geometry_msgs::Pose2D> MotionVisualizerDrums::RandomSwitch(geometry_msgs::Pose2D& motionValuesBasic, geometry_msgs::Pose2D& motionValuesDifferential)
//{
  // TODO: think about running this before low pass filtering..
  // TODO: in certain time instances create random number based selection of entries

  // If not called stick to class variable based numbers

  // DR params: time in s for redist sampling. Probability for redist sampling to happen. Number of running motors x of 6

  // TODO: Fill this in appropriately.
  //geometry_msgs::Pose2D pose2D1 = motionValuesBasic;
  //geometry_msgs::Pose2D pose2D2 = motionValuesDifferential;
  //return std::make_tuple(pose2D1, pose2D2);
//}


void MotionVisualizerDrumsOpenCV::drCallback(simple_kinect_motion_visualizer::VisualizationConfig &config, uint32_t level) {

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
  preLPFMusicGainDifferential_ = config.preLPFMusicGainDifferential;




  // TODO: Add lpfIntensitythreshold
  lpfRedIntensityThreshold_ = config.lpfRedIntensityThreshold;
  lpfGreenIntensityThreshold_ = config.lpfGreenIntensityThreshold;
  lpfBlueIntensityThreshold_ = config.lpfBlueIntensityThreshold;

  // Motor multiplier.
  motorMultiplier_ = config.motorMultiplier;

  // Motor multiplier differential.
  motorMultiplierDifferential_ = config.motorMultiplierDifferential;

  // Simpler Differential Version.
  LPFgainSimpleDiff_ = config.LPFgainSimpleDiff;
  sensitivitySimpleDiff_ = config.sensitivitySimpleDiff;

  // Individual Settings.
  sensitivityBasicX_ = config.sensitivityBasicX;
  sensitivityBasicY_ = config.sensitivityBasicY;
  sensitivityBasicTHETA_ = config.sensitivityBasicTHETA;

  maxVelocityBasicX_ = config.maxVelocityBasicX;
  maxVelocityBasicY_ = config.maxVelocityBasicY;
  maxVelocityBasicTHETA_ = config.maxVelocityBasicTHETA;

  lpfGainBasicX_ = config.lpfGainBasicX;
  lpfGainBasicY_ = config.lpfGainBasicY;
  lpfGainBasicTHETA_ = config.lpfGainBasicTHETA;

  sensitivityDiffX_ = config.sensitivityDiffX;
  sensitivityDiffY_ = config.sensitivityDiffY;
  sensitivityDiffTHETA_ = config.sensitivityDiffTHETA;

  maxVelocityDiffX_ = config.maxVelocityDiffX;
  maxVelocityDiffY_ = config.maxVelocityDiffY;
  maxVelocityDiffTHETA_ = config.maxVelocityDiffTHETA;

  lpfGainDiffX_ = config.lpfGainDiffX;
  lpfGainDiffY_ = config.lpfGainDiffY;
  lpfGainDiffTHETA_ = config.lpfGainDiffTHETA;
}

//void MotionVisualizerDrums::cameraInfoCallback(const sensor_msgs::CameraInfo& cameraInfo)
//{
//    geometry_msgs::Pose2D valuesForLED;
//    valuesForLED.x = 0.8 * OldLEDlpfX_ + 0.2 * basicMotorVelocityX_;
//    valuesForLED.y = 0.8 * OldLEDlpfY_ + 0.2 * basicMotorVelocityY_;
//    valuesForLED.theta = 0.8 * OldLEDlpfTHETA_ + 0.2 * basicMotorVelocityTHETA_;
//
//    OldLEDlpfX_ = valuesForLED.x;
//    OldLEDlpfY_ = valuesForLED.y;
//    OldLEDlpfTHETA_ = valuesForLED.theta;
//    MusicValuePublisherForLED_.publish(valuesForLED);
//}

MotionVisualizerDrumsOpenCV::~MotionVisualizerDrumsOpenCV()
{

  //cout << "In destructor: " << this->lpfGainUp_ << endl;
//  // New added to write data to file for learning.
//  map_.writeDataFileForParameterLearning();

//  fusionServiceQueue_.clear();
//  fusionServiceQueue_.disable();
//  nodeHandle_.shutdown();
//  fusionServiceThread_.join();
}
