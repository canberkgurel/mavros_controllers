//  March/2020, Auterion, Jaeyoung Lim, jaeyoung@auterion.com

#include "parameter_estimator/parameter_estimator.h"

using namespace Eigen;
using namespace std;
//Constructor
ParameterEstimator::ParameterEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &ParameterEstimator::cmdloopCallback, this); // Define timer for constant loop rate

}
ParameterEstimator::~ParameterEstimator() {
  //Destructor
}

void ParameterEstimator::cmdloopCallback(const ros::TimerEvent& event){
  std::cout << "Fuuuuck " << std::endl;

}