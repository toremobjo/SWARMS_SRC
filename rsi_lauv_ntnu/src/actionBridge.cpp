// ROS headers
#include <ros/ros.h>

// Local headers
#include <bridgeLauv.hpp>
#include <utilFctn.hpp>
#include <actionBridge.hpp>

static std::string nodeName = "bridgeLauvNode";

namespace rsilauv{

  ActionBridge::ActionBridge(ros::NodeHandle& node_handle,
    std::string& nodeName,
    std::string& serverAddr,
    std::string& serverPort,
    std::string& vehicleName):
    nh_(node_handle),
    nodeName_(nodeName),
    serverAddr_(serverAddr),
    serverPort_(serverPort),
    tcp_client_(NULL),
    tcp_client_thread_(NULL),
    vehicleName_(vehicleName),
    vehicleId_(-1),
    action_id_(1), //temporary action ID
    vehicleService_(false),
    nbActions_(0),
    lastState_(0),
    flagInitPlanOutCome_(true),
    flagEntity_(false)
    {
      gotoServer = nh_.advertiseService("runGOTO_WAYPOINT",
      &ActionBridge::runGotoWaypoint,this);
    }

    bool ActionBridge::runGotoWaypoint(g2s_interface::runGOTO_WAYPOINT::Request &req,
      g2s_interface::runGOTO_WAYPOINT::Response &res){
      return 1;
    }

    bool ActionBridge::isConnectedDetermined(void)
    {
      if (tcp_client_->isConnected())
      {
        if (isVehicleIdDetermined())
          return true;
        else
        {
          ROS_WARN("[%s] Vehicle not determined.",nodeName_.c_str());
          return false;
        }
      }
      else
      {
        ROS_WARN("[%s] Bridge not connected.",nodeName_.c_str());
        return false;
      }
    }

    bool ActionBridge::isVehicleIdDetermined(){
      return (vehicleId_>0);
    }

    void ActionBridge::start(void)
    {
      stop();
      tcp_client_ = new ros_imc_broker::TcpLink(boost::bind(&Bridge::messageIn, this, _1));
      tcp_client_->setServer(serverAddr_, serverPort_);
      tcp_client_thread_ = new boost::thread(boost::ref(*tcp_client_));
    }
    // Copy/Past form OMST
    void ActionBridge::stop(void)
    {
      if (tcp_client_thread_ == NULL)
        return;
      tcp_client_thread_->interrupt();
      tcp_client_thread_->join();
      delete tcp_client_thread_;
      tcp_client_thread_ = NULL;
      delete tcp_client_;
      tcp_client_ = NULL;
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
