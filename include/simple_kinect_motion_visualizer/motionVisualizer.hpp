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
#include "simple_kinect_motion_visualizer/VisualizationConfig.h"

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
  double lpfGainUp_;
  double lpfGainDown_;

  private:

  void edgeDetectionImageCallback(const sensor_msgs::Image& imageEdgeDetection);

  // List of sensor images (for history)
  std::vector<sensor_msgs::Image> edgeDetectionImageHistory;

  ros::Subscriber edgeDetectionImageSubscriber_;
  ros::Publisher coloredImagePublisher_;

  ros::NodeHandle nodeHandle_;

  // Temporary image for low pass filtering
  sensor_msgs::Image outputImageTemp_;
  // Trigger for low pass filtering.
  bool lpfTrigger_;



};

