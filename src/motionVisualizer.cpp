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

  coloredImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>("/colored_image2", 1);
  coloredCombinedImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>("/colored_image_combined2", 5);

  MusicValuePublisher_ = nodeHandle_.advertise<geometry_msgs::Pose2D>("music_values", 1);
  DifferentialMusicValuePublisher_ = nodeHandle_.advertise<geometry_msgs::Pose2D>("music_values_differential", 1);

  cogPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("cog_keyvalues", 5);

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
}


void MotionVisualizer::edgeDetectionImageCallback(
    const sensor_msgs::Image& imageEdgeDetection)
{

  // Tests:
  //if (alternator_) {
  //    alternator_ = false;
  //    return;
  //}
  //else alternator_ = true;


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


  // Set number of considered images for blurring
  if (edgeDetectionImageHistory_.size() > 1) edgeDetectionImageHistory_.erase(edgeDetectionImageHistory_.begin()); // Hacked a 2 in here, history sizes will have no effect anymore
  edgeDetectionImageHistory_.push_back(imageEdgeDetection);

  if (outputImages_.size() > 1) outputImages_.erase(outputImages_.begin()); // Hacked a 2 in here, history sizes will have no effect anymore
  outputImages_.push_back(outputImage);

  //std::cout << "edgeDetectionImageHistory__size: " << edgeDetectionImageHistory_.size() << std::endl;


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
    if (totalDifference > musicIntensityTreshold) {
        totalDifferenceMusicValue += totalDifference;
        if (pixelXAxis <= 0.5) totalDifferenceMusicValueLeft += totalDifference;
        if (pixelXAxis >= 0.5) totalDifferenceMusicValueRight += totalDifference;
    }


    //ROS_INFO("Filter gain up: %f", lpfGainUp_);
    //ROS_INFO("Filter gain down: %f", this->lpfGainDown_);

    // TODO: change ordering of if checks:


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

                outputImages_[1].data[4*i+1] = fmin(255, greenGain_ * totalDifference);
                //if (totalDifference <= greenIntensityThreshold_) outputImages_[1].data[4*i+1] = 0;
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

                outputImages_[1].data[4*i+2] = fmin(255, blueGain_ * totalDifference);
                //if (totalDifference <= blueIntensityThreshold_) outputImages_[1].data[4*i+2] = 0;

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



  if (outputImages_.size() > 1) {
    coloredImagePublisher_.publish(outputImages_[1]);
  }


  // Option to publish the combined image.
  if (generateCombinedImage_) coloredCombinedImagePublisher_.publish(outputImageCombined);

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


    std::cout << "musicValueTotal 1: " << musicValueTotal << std::endl;
    std::cout << "DifferenceMusicValue 1 squared: " << totalDifferenceMusicValue * totalDifferenceMusicValue << std::endl;
    // 


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

    std::cout << "musicValueTotal: " << musicValueTotal << std::endl;

    // Pre LPFing
    if (preLPFTrigger_ == true) musicValueTotal = musicValueTotal * (1.0 - preLPFMusicGain_) + oldMusicValue_ * preLPFMusicGain_;
    oldMusicValue_ = musicValueTotal;

    // PreLPFing of differential version
    if (preLPFTrigger_ == true) musicTwistDifferential = musicTwistDifferential * (1.0 - preLPFMusicGainDifferential_) + musicTwistOldFiltering_ * preLPFMusicGainDifferential_;
    musicTwistOldFiltering_ = musicTwistDifferential;



    // Pre LPF leftright
    if (leftRight) {
        if (preLPFTrigger_ == true) musicLeft = musicLeft * (1.0 - preLPFMusicGain_) + oldMusicValueLeft_ * preLPFMusicGain_;
        oldMusicValueLeft_ = musicLeft;
        if (preLPFTrigger_ == true) musicRight = musicRight * (1.0 - preLPFMusicGain_) + oldMusicValueRight_ * preLPFMusicGain_;
        oldMusicValueRight_ = musicRight;

        //Differential Version
        if (preLPFTrigger_ == true) musicLeftDifferential = musicLeftDifferential * (1.0 - preLPFMusicGainDifferential_) + musicLeftOldFiltering_ * preLPFMusicGainDifferential_;
        musicLeftOldFiltering_ = musicLeftDifferential;
        if (preLPFTrigger_ == true) musicRightDifferential = musicRightDifferential * (1.0 - preLPFMusicGainDifferential_) + musicRightOldFiltering_ * preLPFMusicGainDifferential_;
        musicRightOldFiltering_ = musicRightDifferential;

    }

    preLPFTrigger_ = true;

    //musicTwist.linear.y = max(min(3.0 * musicTwist.x, 0.5), 0.0);

    geometry_msgs::Pose2D musicTwist;



    musicTwist.x = (int)max(min(0 + motorMultiplier_ * (musicValueTotal - lowerBound_), 800.0), 0.0);
    musicTwist.y = (int)max(min(0 + motorMultiplier_ * (musicLeft - lowerBound_), 800.0), 0.0);
    musicTwist.theta = (int)max(min(0 + motorMultiplier_ * (musicRight - lowerBound_), 800.0), 0.0);

    std::cout << "musicValueTotal just before publishing: " << musicValueTotal << " musicleft: " << musicLeft << " musicRight: " << musicRight << std::endl;



    // TODO: consider using output values for differential calculation..

    // Ouput Values directly.

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
        diffX = fabs((musicTwist.x - XOldForDiff_) * sensitivitySimpleDiff_);
        diffY = fabs((musicTwist.y - YOldForDiff_) * sensitivitySimpleDiff_);
        diffTHETA = fabs((musicTwist.theta - THETAOldForDiff_) * sensitivitySimpleDiff_);

        // Diff LPF Loop:
        if (diffLPFTrigger_ == true)
          {
            geometry_msgs::Pose2D musicSimplerDifferential;
            musicSimplerDifferential.x = (int)max(min(LPFgainSimpleDiff_ * XOldDiffForLPF_ + (1.0 - LPFgainSimpleDiff_) * diffX, 800.0), 0.0);
            musicSimplerDifferential.y = (int)max(min(LPFgainSimpleDiff_ * YOldDiffForLPF_ + (1.0 - LPFgainSimpleDiff_) * diffY, 800.0), 0.0);
            musicSimplerDifferential.theta = (int)max(min(LPFgainSimpleDiff_ * THETAOldDiffForLPF_ + (1.0 - LPFgainSimpleDiff_) * diffTHETA, 800.0), 0.0);
            DifferentialMusicValuePublisher_.publish(musicSimplerDifferential);
          }

        // Assign Old LPF Diff values
        XOldDiffForLPF_ = diffX;
        YOldDiffForLPF_ = diffY;
        THETAOldDiffForLPF_ = diffTHETA;


        diffLPFTrigger_ = true;
      }

      // Assign Old values
      XOldForDiff_ = musicTwist.x;
      YOldForDiff_ = musicTwist.y;
      THETAOldForDiff_ = musicTwist.theta;


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

    MusicValuePublisher_.publish(musicTwist);

  }
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
