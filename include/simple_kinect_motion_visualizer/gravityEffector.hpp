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

#include "simple_kinect_motion_visualizer/VisualizationConfig.h"
//#include <effects/builtin/filtereffect.h>

class GravityEffector
{
  public:

  // Constructor
  GravityEffector(ros::NodeHandle& nodeHandle);

  // Destructor
  ~GravityEffector();

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

  // Music dr parameters.
  bool quadraticCorrelation_;
  double gainDivider_;
  double minusTerm_;
  double lowerBound_;

  // PreLPF values
  double preLPFMusicGain_;
  double oldMusicValue_;
  bool preLPFTrigger_;

  // cog Characterizations.
  double cogX_;
  double cogY_;
  int accumulatedDifference_;

  double lpfcogX_;
  double lpfcogY_;
  int lpfaccumulatedDifference_;
  double oldcogX_;
  double oldcogY_;
  int oldaccumulatedDifference_;
  bool lpfcogTrigger_;

  private:

  void inputImageCallback(const sensor_msgs::Image& inputImage);

  void cogCallback(geometry_msgs::Twist cogKeyvalues);


  // List of sensor images (for history)
  //std::vector<sensor_msgs::Image> edgeDetectionImageHistory;

  ros::Subscriber inputImageSubscriber_;
  //ros::Publisher coloredCombinedImagePublisher_;
  ros::Publisher outputImagePublisher_;

  ros::Subscriber cogSubscriber_;

  //ros::Publisher cogPublisher_;

  //ros::Publisher MusicValuePublisher_;

  ros::NodeHandle nodeHandle_;

  // Temporary image for low pass filtering
  //sensor_msgs::Image outputImageTemp_;
  // Trigger for low pass filtering.
  //bool lpfTrigger_;


  //bool generateCombinedImage_;



};

