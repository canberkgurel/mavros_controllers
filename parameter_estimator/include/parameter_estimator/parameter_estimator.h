//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef PARAMETER_ESTIMATOR_H
#define PARAMETER_ESTIMATOR_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>

#include <Eigen/Dense>


using namespace std;
using namespace Eigen;

class ParameterEstimator
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Timer cmdloop_timer_;

    void cmdloopCallback(const ros::TimerEvent& event);

  public:
    ParameterEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ ParameterEstimator();

};


#endif
