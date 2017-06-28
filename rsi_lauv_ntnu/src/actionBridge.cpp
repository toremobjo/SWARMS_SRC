// ROS headers
#include <ros/ros.h>

// Local headers
#include <bridgeConnection.hpp>
#include <utilFctn.hpp>
#include <actionBridge.hpp>

static std::string nodeName = "bridgeLauvNode";

namespace rsilauv{

  ActionBridge::ActionBridge():/*
    nh_(node_handle),
    nodeName_(nodeName),
    serverAddr_(serverAddr),
    serverPort_(serverPort),*/
    //tcp_client_(NULL),
    //tcp_client_thread_(NULL),
    action_id_(1), //temporary action ID
    lastState_(0)
    {
      gotoServer = nh_.advertiseService("runGOTO_WAYPOINT",
      &ActionBridge::runGotoWaypoint,this);
    }

    bool ActionBridge::runGotoWaypoint(g2s_interface::runGOTO_WAYPOINT::Request &req,
      g2s_interface::runGOTO_WAYPOINT::Response &res){
      return 1;
    }


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, nodeName, ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  //ROS_INFO("[%s] Here I am ...",nodeName.c_str());

  std::string serverAddr, serverPort, vehicleName;
  utilfctn::getParam("~server_addr",serverAddr);
  utilfctn::getParam("~server_port",serverPort);
  utilfctn::getParam("~vehicle_name",vehicleName);

  //rsilauv::Bridge bridge(nh,nodeName,serverAddr,serverPort,vehicleName);

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
