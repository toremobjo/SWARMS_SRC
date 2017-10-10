//#ifndef DRSI_LAUV_NTNU_BRIDGE_HPP_INCLUDED_
//#define DRSI_LAUV_NTNU_BRIDGE_HPP_INCLUDED_

// ROS headers
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

// RSI MESSAGE
#include "g2s_interface/robotStatus.h"
#include "g2s_interface/robotSituation.h"
#include "g2s_interface/robotAlarm.h"
#include "g2s_interface/endOfAction.h"
#include "g2s_interface/waterData.h"
#include "g2s_interface/environmentData.h"

// RSI SERVICE
#include "g2s_interface/abort_Action.h"
#include "g2s_interface/actionStatus.h"
#include "g2s_interface/activateACOUSTIC.h"
#include "g2s_interface/activateCAMERA.h"
#include "g2s_interface/activateGPS.h"
#include "g2s_interface/activateLASER.h"
#include "g2s_interface/activateLIGHT.h"
#include "g2s_interface/activateSONAR.h"
#include "g2s_interface/activateUSBL.h"
#include "g2s_interface/activateWIFI.h"
#include "g2s_interface/activityStatus.h"
#include "g2s_interface/deactivateEqpt.h"
#include "g2s_interface/powerStatus.h"
#include "g2s_interface/resetEqpt.h"
#include "g2s_interface/resume_Action.h"
#include "g2s_interface/runASCEND_GPS.h"
#include "g2s_interface/runASCEND.h"
#include "g2s_interface/runCIRCLE_USBL.h"
#include "g2s_interface/runDIVE.h"
#include "g2s_interface/runFOLLOW_ROW.h"
#include "g2s_interface/runFOLLOW_STRUCTURE.h"
#include "g2s_interface/runGOTO_WAYPOINT.h"
#include "g2s_interface/runGRASP_OBJECT.h"
#include "g2s_interface/runSONAR_SCANNING.h"
#include "g2s_interface/runTRACK_TARGET.h"
#include "g2s_interface/runVIDEO_ACQUISITION.h"
#include "g2s_interface/runWAIT.h"
#include "g2s_interface/stopEqptAction.h"
#include "g2s_interface/suspend_Action.h"

// Boost headers.
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>

namespace drsilauv
{
  struct Water
  {
    float temperature;
    float salinity;
    float conductivity;
    float pressure;
    float soundSpeed;
    float turbidity;
    float ph;
    float levelH2S;
    float levelDVL;
  };
  struct Situation
  {
    float x, y, z; // x: North, east and down offsets (s.t LLH).
    double lat, lon, height; // Latitudinal, longditudinal and height position estimate.
    double latlin, lonlin;
    float phi, theta, psi; // Euler angles for roll, pitch and yaw.
    float u, v, w; // Velocities in the body-fixed coordinate frame.
    float vx, vy, vz; // Velocities in the earth-fixed coordinate frame.
    float p, q, r; //Angular velocity in roll, pitch and yaw (body-fixed).
    float depth; //Depth below surface.
    float altitude;
    int   sequence;
  };
  struct Fuel
  {
    float level;
    float confInter;
  };
  struct Action
  {
    ros::Time requestTime;
    int actionNumber;
    //DUNE::IMC::PlanControl actionPlan;
    bool success;
  };
  class dBridge
  {
  public:
    //Constructor and Destructor
    dBridge(ros::NodeHandle& node_handle, std::string& nodeName);
    ~dBridge(void){stop();}

    // Messages ot for dummy interface
    void messageOut(const ros::TimerEvent& event){;};
    void stop(void);
  private:
    ros::NodeHandle nh_;
    std::string& nodeName_;

    //Message publishers
    ros::Publisher pub1;
    ros::Publisher pub2;
    ros::Publisher pub3;
    ros::Publisher pub4;
    ros::Publisher pub5;
    ros::Publisher pub6;

    //Service Servers
    ros::ServiceServer ser1;
    ros::ServiceServer ser2;
    ros::ServiceServer ser3;
    ros::ServiceServer ser4;
    ros::ServiceServer ser5;
    ros::ServiceServer ser6;
    ros::ServiceServer ser7;
    ros::ServiceServer ser8;
    ros::ServiceServer ser9;
    ros::ServiceServer ser10;
    ros::ServiceServer ser11;
    ros::ServiceServer ser12;
    ros::ServiceServer ser13;
    ros::ServiceServer ser14;
    ros::ServiceServer ser15;
    ros::ServiceServer ser16;
    ros::ServiceServer ser17;
    ros::ServiceServer ser18;
    ros::ServiceServer ser19;
    ros::ServiceServer ser20;
    ros::ServiceServer ser21;
    ros::ServiceServer ser22;
    ros::ServiceServer ser23;
    ros::ServiceServer ser24;
    ros::ServiceServer ser25;
    ros::ServiceServer ser26;
    ros::ServiceServer ser27;
    ros::ServiceServer ser28;
    ros::ServiceServer ser29;

    //Service functions
    //Abort spesific  action, do nothing after.
    bool runAbortAction(g2s_interface::abort_Action::Request &req,
      g2s_interface::abort_Action::Response &res);

    //Get action status
    bool getActionStatus(g2s_interface::actionStatus::Request &req,
      g2s_interface::actionStatus::Response &res);

    //Activate acoustic communication
    bool runActivateAcoustic(g2s_interface::activateACOUSTIC::Request &req,
      g2s_interface::activateACOUSTIC::Response &res);

    //Activate vehicle camera
    bool runActivateCamera(g2s_interface::activateCAMERA::Request &req,
      g2s_interface::activateCAMERA::Response &res);

    //Activate GPS, on LAUV GPS is always activated at the suface.
    bool runActivateGPS(g2s_interface::activateGPS::Request &req,
      g2s_interface::activateGPS::Response &res);

    // Activate laser, unclear what or wich kind of laser ??
    bool runActivateLASER(g2s_interface::activateLASER::Request &req,
      g2s_interface::activateLASER::Response &res);

    bool runActivateLIGHT(g2s_interface::activateLIGHT::Request &req,
      g2s_interface::activateLIGHT::Response &res);

    bool runActivateSONAR(g2s_interface::activateSONAR::Request &req,
      g2s_interface::activateSONAR::Response &res);

    bool runActivateUSBL(g2s_interface::activateUSBL::Request &req,
      g2s_interface::activateUSBL::Response &res);

    bool runActivateWIFI(g2s_interface::activateWIFI::Request &req,
      g2s_interface::activateWIFI::Response &res);

    bool runActivityStatus(g2s_interface::activityStatus::Request &req,
      g2s_interface::activityStatus::Response &res);

    bool runDeactivateEqpt(g2s_interface::deactivateEqpt::Request &req,
      g2s_interface::deactivateEqpt::Response &res);

    bool getPowerStatus(g2s_interface::powerStatus::Request &req,
      g2s_interface::powerStatus::Response &res);

    bool runResetEqpt(g2s_interface::resetEqpt::Request &req,
      g2s_interface::resetEqpt::Response &res);

    bool runResume_Action(g2s_interface::resume_Action::Request &req,
      g2s_interface::resume_Action::Response &res);

    bool runRunASCEND_GPS(g2s_interface::runASCEND_GPS::Request &req,
      g2s_interface::runASCEND_GPS::Response &res);

    bool runRunASCEND(g2s_interface::runASCEND::Request &req,
      g2s_interface::runASCEND::Response &res);

    bool runCIRCLE_USBL(g2s_interface::runCIRCLE_USBL::Request &req,
      g2s_interface::runCIRCLE_USBL::Response &res);

    bool runDIVE(g2s_interface::runDIVE::Request &req,
      g2s_interface::runDIVE::Response &res);

    bool runFOLLOW_ROW(g2s_interface::runFOLLOW_ROW::Request &req,
      g2s_interface::runFOLLOW_ROW::Response &res);

    bool runFOLLOW_STRUCTURE(g2s_interface::runFOLLOW_STRUCTURE::Request &req,
      g2s_interface::runFOLLOW_STRUCTURE::Response &res);

    bool runGOTO_WAYPOINT(g2s_interface::runGOTO_WAYPOINT::Request &req,
      g2s_interface::runGOTO_WAYPOINT::Response &res);

    bool runGRASP_OBJECT(g2s_interface::runGRASP_OBJECT::Request &req,
      g2s_interface::runGRASP_OBJECT::Response &res);

    bool runSONAR_SCANNING(g2s_interface::runSONAR_SCANNING::Request &req,
      g2s_interface::runSONAR_SCANNING::Response &res);

    bool runTRACK_TARGET(g2s_interface::runTRACK_TARGET::Request &req,
      g2s_interface::runTRACK_TARGET::Response &res);

    bool runVIDEO_ACQUISITION(g2s_interface::runVIDEO_ACQUISITION::Request &req,
      g2s_interface::runVIDEO_ACQUISITION::Response &res);

    bool runWAIT(g2s_interface::runWAIT::Request &req,
      g2s_interface::runWAIT::Response &res);

    bool stopEqptAction(g2s_interface::stopEqptAction::Request &req,
      g2s_interface::stopEqptAction::Response &res);

    bool suspend_Action(g2s_interface::suspend_Action::Request &req,
      g2s_interface::suspend_Action::Response &res);
  };
}
