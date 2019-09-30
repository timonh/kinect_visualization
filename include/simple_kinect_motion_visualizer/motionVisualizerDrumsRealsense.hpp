//// Boost
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>

// STL
#include <string>
#include <math.h>
#include <limits>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

#include <dynamic_reconfigure/server.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"

#include "simple_kinect_motion_visualizer/VisualizationConfig.h"
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include <image_transport/image_transport.h>

//#include <effects/builtin/filtereffect.h>

class MotionVisualizerDrumsRealsense
{
  public:

  // Constructor
  MotionVisualizerDrumsRealsense(ros::NodeHandle& nodeHandle);

  // Destructor
  ~MotionVisualizerDrumsRealsense();

  void drCallback(simple_kinect_motion_visualizer::VisualizationConfig &config, uint32_t level);

  // Dynamic Reconfigure Parameters.
  double redHistorySize_;
  double redGain_;
  double redIntensityThreshold_;
  double greenHistorySize_;
  double greenGain_;
  double greenIntensityThreshold_;
  double blueHistorySize_;
  double blueGain_;
  double blueIntensityThreshold_;
  double redlpfGainUp_;
  double redlpfGainDown_;
  double greenlpfGainUp_;
  double greenlpfGainDown_;
  double bluelpfGainUp_;
  double bluelpfGainDown_;

  // LPF intensity threholds for efficiency.
  double lpfRedIntensityThreshold_;
  double lpfGreenIntensityThreshold_;
  double lpfBlueIntensityThreshold_;

  // Music dr parameters.
  bool quadraticCorrelation_;
  double gainDivider_;
  double minusTerm_;
  double lowerBound_;

  // Motor Params.
  int motorMultiplier_;
  int motorMultiplierDifferential_;

  // PreLPF values.
  double preLPFMusicGain_;
  double oldMusicValue_;
  bool preLPFTrigger_;

  // PreLPF values Differential.
  double preLPFMusicGainDifferential_;

  // Left Right Music Value distinction.
  double oldMusicValueLeft_;
  double oldMusicValueRight_;

  // Differential derivative calculation utils.
  bool differentiationtrigger_;
  double musicTwistOldNoFilter_;
  double musicLeftOldNoFilter_;
  double musicRightOldNoFilter_;

  double musicTwistOldFiltering_;
  double musicLeftOldFiltering_;
  double musicRightOldFiltering_;

  // Simpler Differential calculation.
  double LPFgainSimpleDiff_;
  double sensitivitySimpleDiff_;
  double XOldForDiff_, YOldForDiff_, THETAOldForDiff_;
  double XOldDiffForLPF_, YOldDiffForLPF_, THETAOldDiffForLPF_;
  bool diffTrigger_;
  bool diffLPFTrigger_;

  // Individual sensitivity settings:
  double sensitivityBasicX_, sensitivityBasicY_, sensitivityBasicTHETA_,
  sensitivityDiffX_, sensitivityDiffY_, sensitivityDiffTHETA_;
  double maxVelocityBasicX_, maxVelocityBasicY_, maxVelocityBasicTHETA_,
  maxVelocityDiffX_, maxVelocityDiffY_, maxVelocityDiffTHETA_;
  double lpfGainBasicX_, lpfGainBasicY_, lpfGainBasicTHETA_,
  lpfGainDiffX_, lpfGainDiffY_, lpfGainDiffTHETA_;

  // 18 Fields Motor velocity.
  int basicMotorVelocityInFieldsArray_[162];
  bool drumActivationInFieldsArray_[162];
  bool drumActivationInFieldsArrayOld_[162];
  int colorizationIntensityArray_[162];

  // Timing for Drum triggereing tests.
  double newDrumTriggerTime_;
  double oldDrumTriggerTime_;

  // Helpers for secondary lpf of LED values.
  double OldLEDlpfX_, OldLEDlpfY_, OldLEDlpfTHETA_;

  // Storage for calculated motor velocity values
  int basicMotorVelocityX_, basicMotorVelocityY_, basicMotorVelocityTHETA_;

  // Bool for preliminary field pattern distribution.
  bool useTinyAdjustable_;



  private:

  void inputImageCallbackFormatTest(const sensor_msgs::ImageConstPtr& inputImage);
  void inputImageCallback(const sensor_msgs::ImageConstPtr& inputImage);

  void depthImageCallback(const sensor_msgs::ImageConstPtr& imageEdgeDetection);

  void cameraInfoCallback(const sensor_msgs::CameraInfo& CameraInfo);

  void themeSwitcherAfterSixteenCallback(const geometry_msgs::Pose2D& themeSwitcherAfterSixteenMsg);

  // Randomizer for motors running.
  std::tuple<geometry_msgs::Pose2D, geometry_msgs::Pose2D> RandomSwitch(geometry_msgs::Pose2D& motionValuesBasic, geometry_msgs::Pose2D& motionValuesDifferential);

  // List of sensor images (for history)
  std::vector<sensor_msgs::Image> inputImageHistory_;

  // List of Output Images (For low pass filtering)
  std::vector<sensor_msgs::Image> outputImages_;

  ros::Subscriber edgeDetectionImageSubscriber_;
  ros::Subscriber depthImageSubscriber_;
  ros::Publisher coloredCombinedImagePublisher_;
  ros::Publisher coloredImagePublisher_;

  // Subscriber for switching themes after 16 takte of loop recording.
  ros::Subscriber themeSwitcherAfterSixteenSubscriber_;


  //image_transport::Subscriber inputImageSubscriber_;
  //image_transport::Publisher ouputImagePublisher_;

  // Drum Message Publisher.
  ros::Publisher drumMessagePublisher_;

  ros::Publisher cogPublisher_;

  ros::Publisher MusicValuePublisher_;
  ros::Publisher DifferentialMusicValuePublisher_;
  ros::Publisher MusicValuePublisherForLED_;

  // Helper subscriber to publish in different frequency within the same node.
  ros::Subscriber cameraInfoSubscriber_;

  ros::NodeHandle nodeHandle_;

  // Temporary image for low pass filtering
  sensor_msgs::Image outputImageTemp_;
  // Trigger for low pass filtering.
  bool lpfTrigger_;

  // Bools for performance tests
  bool generateCombinedImage_;
  bool getCOG_;
  bool publishMusicTwist_;

  bool alternator_;

  // Chose if applying LPFing (potentially increasing rate)
  bool applyLPF_;

  // Parameters for efficient Mean Distance approximation from depth image
  double totalDistanceAccumulation_;
  int noOfPixelsConsidered_;
  int blockingCounter_;
  double meanDistanceThreshold_;
  double oldMeanDistance_;
  ros::Publisher distanceBasedThemeSwitchingPublisher_;
};

