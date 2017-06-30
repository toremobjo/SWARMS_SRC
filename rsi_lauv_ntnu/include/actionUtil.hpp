#ifndef ACTION_UTIL_HPP_INCLUDED_
#define ACTION_UTIL_HPP_INCLUDED_

// ROS headers
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>


#include "rsi_lauv_ntnu/testMsgRsiLauv.h"
#include "rsi_lauv_ntnu/testSrvRsiLauv.h"

// RSI headers
// #include <rgi_rsi/.hpp>
// #include <include/rsi.h>
// RSI MESSAGE
#include "g2s_interface/robotStatus.h"
#include "g2s_interface/robotSituation.h"
// #include "g2s_interface/robotAlarm.h"
#include "g2s_interface/endOfAction.h"
#include "g2s_interface/waterData.h"
#include "g2s_interface/environmentData.h"

// RSI SERVICE
#include "g2s_interface/powerStatus.h"
#include "g2s_interface/runGOTO_WAYPOINT.h"
#include "g2s_interface/abort_Action.h"
#include "g2s_interface/actionStatus.h"

// Boost headers.
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Local headers
#include <TcpLink.hpp>
#include <utilFctn.hpp>
#include <imcIdDefs.h>

namespace actionUtil
{
  int actionUtilTest()
  {
    return 5;
  }

  DUNE::IMC::PlanControl actionUtilMakePCGoto(g2s_interface::runGOTO_WAYPOINT::Request &req,
    g2s_interface::runGOTO_WAYPOINT::Response &res)
  {
      std::string plan_id_ = "swarms";
      float desiredSpeed;

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
    }
}
#endif
