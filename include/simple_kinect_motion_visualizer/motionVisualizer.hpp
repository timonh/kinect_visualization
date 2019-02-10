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


class MotionVisualizer
{
  public:

  // Constructor
  MotionVisualizer(ros::NodeHandle& nodeHandle);

  // Destructor
  ~MotionVisualizer();

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

