// ROS headers
#include <ros/ros.h>

// Local headers
#include <bridgeLauv.hpp>
#include <utilFctn.hpp>

static std::string nodeName = "bridgeLauvNode";

int main(int argc, char** argv)
{
  ros::init(argc, argv, nodeName, ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  //ROS_INFO("[%s] Here I am ...",nodeName.c_str());

  std::string serverAddr, serverPort, vehicleName;
  utilfctn::getParam("~server_addr",serverAddr);
  utilfctn::getParam("~server_port",serverPort);
  utilfctn::getParam("~vehicle_name",vehicleName);

  rsilauv::Bridge bridge(nh,nodeName,serverAddr,serverPort,vehicleName);

  ros::spin();

  // ros::Rate loopRate(1); //Hz
  // utilfctn::TicToc chrono1;
  // chrono1.start();
  // while (ros::ok())
  // {
  //   // chrono1.stop();
  //   // ROS_INFO("[%s] Chrono %.4f Sec (%.2f Hz)",nodeName.c_str(),
  //   //   chrono1.getDtSec(),1/chrono1.getDtSec());
  //   // chrono1.start();
  //
  //   if (true)
  //   {
  //     bridge.messageOut();
  //   }
  //   ros::spinOnce();
  //   loopRate.sleep();
  // }
  return 0;
}
