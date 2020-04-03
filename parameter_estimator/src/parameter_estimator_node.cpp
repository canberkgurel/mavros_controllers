//  March/2020, Auterion, Jaeyoung Lim, jaeyoung@auterion.com

#include "parameter_estimator/parameter_estimator.h"

//using namespace RAI;
int main(int argc, char** argv) {
  ros::init(argc,argv,"parameter_estimator");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ParameterEstimator *parameter_estimator = new ParameterEstimator(nh, nh_private);

  ros::spin();
  return 0;
}
