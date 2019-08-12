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

#include <dynamic_reconfigure/server.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"

#include "simple_kinect_motion_visualizer/VisualizationConfig.h"
//#include <effects/builtin/filtereffect.h>

class MotionVisualizer
{
  public:

  // Constructor
  MotionVisualizer(ros::NodeHandle& nodeHandle);

  // Destructor
  ~MotionVisualizer();

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


  private:

  void edgeDetectionImageCallback(const sensor_msgs::Image& imageEdgeDetection);

  // Randomizer for motors running.
  std::tuple<geometry_msgs::Pose2D, geometry_msgs::Pose2D> RandomSwitch(geometry_msgs::Pose2D& motionValuesBasic, geometry_msgs::Pose2D& motionValuesDifferential);

  // List of sensor images (for history)
  std::vector<sensor_msgs::Image> edgeDetectionImageHistory_;

  // List of Output Images (For low pass filtering)
  std::vector<sensor_msgs::Image> outputImages_;

  ros::Subscriber edgeDetectionImageSubscriber_;
  ros::Publisher coloredCombinedImagePublisher_;
  ros::Publisher coloredImagePublisher_;

  ros::Publisher cogPublisher_;

  ros::Publisher MusicValuePublisher_;
  ros::Publisher DifferentialMusicValuePublisher_;

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
};

