// ROS headers
#include <ros/ros.h>

// Local headers
#include <bridgeConnection.hpp>
#include <utilFctn.hpp>
#include <actionBridge.hpp>

namespace rsilauvActions{

  /*ActionBridge::ActionBridge()
    {
      //gotoServer = nh_.advertiseService("runGOTO_WAYPOINT",
      //&ActionBridge::runGotoWaypoint,this);
    }


    bool ActionBridge::runGotoWaypoint(g2s_interface::runGOTO_WAYPOINT::Request &req,
      g2s_interface::runGOTO_WAYPOINT::Response &res){
      return 1;
    }

    DUNE::IMC::PlanControl ActionBridge::actionBridgeGoto(g2s_interface::runGOTO_WAYPOINT::Request &req,
      g2s_interface::runGOTO_WAYPOINT::Response &res)
    {
      DUNE::IMC::PlanControl pc;
      pc.plan_id = plan_id_;
      pc.op = DUNE::IMC::PlanControl::PC_START;
      pc.type = DUNE::IMC::PlanControl::PC_REQUEST;
      pc.request_id = 1000;

      DUNE::IMC::Goto manGoto;
      DUNE::IMC::PlanManeuver pm;
      DUNE::IMC::PlanSpecification ps;

      //calculate from relative position(NED) in relation to Fixed point.
      geometry_msgs::Point desiredPoint;
      desiredPoint = req.waypointPosition;
      float deltaLat = desiredPoint.y/6386651.041660708; // divided by meters per radian latitude in Trondheim
      float deltaLon = desiredPoint.x/2862544.348782668; // divided by meters per radian longditude in Trondheim

      //ROS_INFO("Desired speed: %f", req.speed);

      if (req.speed > 0.01)
      {
        desiredSpeed = req.speed;
      }else{
        desiredSpeed = 1.6;
      }

      // Goto
      manGoto.lat = 1.1072639824284860 + deltaLat; //todo: change to proper zero-point in time
      manGoto.lon = 0.1806556449351842 + deltaLon;
      manGoto.z = desiredPoint.z;
      manGoto.z_units = DUNE::IMC::Z_DEPTH;
      //manGoto.yaw = 3.14; // todo: implement desired attitude at end of goto
      manGoto.speed = desiredSpeed;
      manGoto.speed_units = DUNE::IMC::SUNITS_METERS_PS;
      manGoto.timeout = 1000;
      manGoto.yaw = req.heading;
      // Maneuver
      pm.maneuver_id = "1";
      pm.data.set(manGoto);
      // Specification
      ps.plan_id = pc.plan_id;
      ps.start_man_id = pm.maneuver_id;
      ps.maneuvers.push_back(pm);

      // PLAN CONTROL
      pc.arg.set(ps);
      return pc;
    }*/
    int actionBridgeTest(){
      int a = 5;
      return a;
    }

    DUNE::IMC::PlanControl actionBridgeGoto(g2s_interface::runGOTO_WAYPOINT::Request &req)
    {
      DUNE::IMC::PlanControl pc;
      pc.plan_id = plan_id_;
      pc.op = DUNE::IMC::PlanControl::PC_START;
      pc.type = DUNE::IMC::PlanControl::PC_REQUEST;
      pc.request_id = 1000;

      DUNE::IMC::Goto manGoto;
      DUNE::IMC::PlanManeuver pm;
      DUNE::IMC::PlanSpecification ps;

      //calculate from relative position(NED) in relation to Fixed point.
      geometry_msgs::Point desiredPoint;
      desiredPoint = req.waypointPosition;
      float deltaLat = desiredPoint.y/6386651.041660708; // divided by meters per radian latitude in Trondheim
      float deltaLon = desiredPoint.x/2862544.348782668; // divided by meters per radian longditude in Trondheim

      //ROS_INFO("Desired speed: %f", req.speed);
      //TODO: Check this, one might want to go at zero speed.
      if (req.speed > 0.01)
      {
        desiredSpeed = req.speed;
      }else{
        desiredSpeed = 1.6;
      }

      // Goto
      manGoto.lat = 1.10725186984 + deltaLat; //1.10724680839 + deltaLat; //todo: change to proper zero-point in time1.10725186984
      manGoto.lon = 0.18064286912 + deltaLon; // 0.18063016312 + deltaLon;
      manGoto.z = desiredPoint.z;
      manGoto.z_units = DUNE::IMC::Z_DEPTH;
      //manGoto.yaw = 3.14; // todo: implement desired attitude at end of goto
      manGoto.speed = desiredSpeed;
      manGoto.speed_units = DUNE::IMC::SUNITS_METERS_PS;
      manGoto.timeout = 1000;
      manGoto.yaw = req.heading;
      // Maneuver
      pm.maneuver_id = "1";
      pm.data.set(manGoto);
      // Specification
      ps.plan_id = pc.plan_id;
      ps.start_man_id = pm.maneuver_id;
      ps.maneuvers.push_back(pm);

      ROS_INFO("runGOTO_WAYPOINT recieved, dispatching.";
      // PLAN CONTROL
      pc.arg.set(ps);
      return pc;
    }
}

/*int main()
{
  /*ros::init(argc, argv, nodeName, ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  //ROS_INFO("[%s] Here I am ...",nodeName.c_str());

  std::string serverAddr, serverPort, vehicleName;
  utilfctn::getParam("~server_addr",serverAddr);
  utilfctn::getParam("~server_port",serverPort);
  utilfctn::getParam("~vehicle_name",vehicleName);

  //rsilauv::Bridge bridge(nh,nodeName,serverAddr,serverPort,vehicleName);

  ros::spin();*/

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
//  return 0;
//}
