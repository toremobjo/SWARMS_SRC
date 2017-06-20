#ifndef UTIL_FCTN_HPP_INCLUDED_
#define UTIL_FCTN_HPP_INCLUDED_

// ROS headers.
#include <ros/ros.h>

namespace utilfctn
{
  template <class ttype>
  void getParam(const std::string name, ttype& value)
  {
    bool res = ros::param::get(name,value);
    if (!res)
    {
      ROS_FATAL_STREAM("I don't get parameter: " << name);
      exit(1);
    }
  }

  int add(int valA, int valB)
  {
    return valA+valB;
  }

  double dt()
  {
    ros::Time begin = ros::Time::now();
    ros::Duration(0.5).sleep();
    ros::Duration dt = ros::Time::now()-begin;
    return dt.toSec();
  }

  class TicToc
  {
  private:
    ros::Time tsta_;
    ros::Time tend_;
  public:
    TicToc(void)
    {
      // Passer par une liste d initialisation?
      tsta_ = ros::Time::now();
      tend_ = ros::Time::now();
    }
    ~TicToc(void)
    {
    }
    void start(void)
    {
      tsta_ = ros::Time::now();
    }
    void stop(void)
    {
      tend_ = ros::Time::now();
    }
    double getDtSec(void)
    {
      ros::Duration dt = tend_-tsta_;
      return dt.toSec();
    }
  };
}
# endif
