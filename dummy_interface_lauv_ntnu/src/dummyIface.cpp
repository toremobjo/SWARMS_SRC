// ROS headers
#include <ros/ros.h>

#include<dummy_interface_lauv_ntnu/dummyIface.hpp>

static std::string nodeName = "dummyIfaceNode";

namespace drsilauv{
  dBridge::dBridge(ros::NodeHandle& node_handle,
     std::string& nodeName):
     nh_(node_handle),
     nodeName_(nodeName)
     //vehicleId_(-1),
     //action_id_(1), //temporary action ID
     //vehicleService_(false),
     //lastState_(0),
     //flagInitPlanOutCome_(true),
     //flagEntity_(false)
     {
       //Message publishers
       pub1 = nh_.advertise<g2s_interface::endOfAction>("endOfAction",1000);
       pub2 = nh_.advertise<g2s_interface::robotAlarm>("robotAlarm",1000);
       pub3 = nh_.advertise<g2s_interface::robotStatus>("robotStatus",1000);
       pub4 = nh_.advertise<g2s_interface::environmentData>("environmentData",1000);
       pub5 = nh_.advertise<g2s_interface::robotSituation>("robotSituation",1000);
       pub6 = nh_.advertise<g2s_interface::waterData>("waterData",1000);

       //Service Servers
       ser1  = nh_.advertiseService("abort_Action", &dBridge::runAbortAction, this);
       ser2  = nh_.advertiseService("actionStatus", &dBridge::getActionStatus, this);
       ser3  = nh_.advertiseService("activateACOUSTIC", &dBridge::runActivateAcoustic, this);
       ser4  = nh_.advertiseService("activateCAMERA", &dBridge::runActivateCamera, this);
       ser5  = nh_.advertiseService("activateGPS", &dBridge::runActivateGPS, this);
       ser6  = nh_.advertiseService("activateLASER", &dBridge::runActivateLASER, this);
       ser7  = nh_.advertiseService("activateLIGHT", &dBridge::runActivateLIGHT, this);
       ser8  = nh_.advertiseService("activateSONAR", &dBridge::runActivateSONAR, this);
       ser9  = nh_.advertiseService("activateUSBL", &dBridge::runActivateUSBL, this);
       ser10 = nh_.advertiseService("activateWIFI", &dBridge::runActivateWIFI, this);
       ser11 = nh_.advertiseService("activityStatus", &dBridge::runActivityStatus, this);
       ser12 = nh_.advertiseService("deactivateEqpt", &dBridge::runDeactivateEqpt, this);
       ser13 = nh_.advertiseService("powerStatus", &dBridge::getPowerStatus, this);
       ser14 = nh_.advertiseService("resetEqpt", &dBridge::runResetEqpt, this);
       ser15 = nh_.advertiseService("resume_Action", &dBridge::runResume_Action, this);
       ser16 = nh_.advertiseService("runASCEND_GPS", &dBridge::runRunASCEND_GPS, this);
       ser17 = nh_.advertiseService("runASCEND", &dBridge::runRunASCEND, this);
       ser18 = nh_.advertiseService("runCIRCLE_USBL", &dBridge::runCIRCLE_USBL, this);
       ser19 = nh_.advertiseService("runDIVE", &dBridge::runDIVE, this);
       ser20 = nh_.advertiseService("runFOLLOW_ROW", &dBridge::runFOLLOW_ROW, this);
       ser21 = nh_.advertiseService("runFOLLOW_STRUCTURE", &dBridge::runFOLLOW_STRUCTURE, this);
       ser22 = nh_.advertiseService("runGOTO_WAYPOINT", &dBridge::runGOTO_WAYPOINT, this);
       ser23 = nh_.advertiseService("runGRASP_OBJECT", &dBridge::runGRASP_OBJECT, this);
       ser24 = nh_.advertiseService("runSONAR_SCANNING", &dBridge::runSONAR_SCANNING, this);
       ser25 = nh_.advertiseService("runTRACK_TARGET", &dBridge::runTRACK_TARGET, this);
       ser26 = nh_.advertiseService("runVIDEO_ACQUISITION", &dBridge::runVIDEO_ACQUISITION, this);
       ser27 = nh_.advertiseService("runWAIT", &dBridge::runWAIT, this);
       ser28 = nh_.advertiseService("stopEqptAction", &dBridge::stopEqptAction, this);
       ser29 = nh_.advertiseService("suspend_Action", &dBridge::suspend_Action, this);
     };

     bool dBridge::runAbortAction(g2s_interface::abort_Action::Request &req,
       g2s_interface::abort_Action::Response &res)
     {
       res.success = true;
       return true;
     }


     bool dBridge::getActionStatus(g2s_interface::actionStatus::Request &req,
       g2s_interface::actionStatus::Response &res)
      {
        //TODO: proper action tracking
        res.actionStatus = "Action Status goes here";
        return true;
      }

      bool dBridge::runActivateAcoustic(g2s_interface::activateACOUSTIC::Request &req,
        g2s_interface::activateACOUSTIC::Response &res)
      {
        res.eqptId = 1;
        return true;
      }

      bool dBridge::runActivateCamera(g2s_interface::activateCAMERA::Request &req,
        g2s_interface::activateCAMERA::Response &res)
      {
        res.eqptId = 2;
        return true;
      }

      bool dBridge::runActivateGPS(g2s_interface::activateGPS::Request &req,
        g2s_interface::activateGPS::Response &res)
      {
        res.eqptId = 3;
        return true;
      }

      bool dBridge::runActivateLASER(g2s_interface::activateLASER::Request &req,
        g2s_interface::activateLASER::Response &res)
      {
        res.eqptId = 4;
        return true;
      }

      bool dBridge::runActivateLIGHT(g2s_interface::activateLIGHT::Request &req,
        g2s_interface::activateLIGHT::Response &res)
      {
        res.eqptId = 5;
        return true;
      }

      bool dBridge::runActivateSONAR(g2s_interface::activateSONAR::Request &req,
        g2s_interface::activateSONAR::Response &res)
      {
        res.eqptId = 6;
        return true;
      }

      bool dBridge::runActivateUSBL(g2s_interface::activateUSBL::Request &req,
        g2s_interface::activateUSBL::Response &res)
      {
        res.eqptId = 7;
        return true;
      }

      bool dBridge::runActivateWIFI(g2s_interface::activateWIFI::Request &req,
        g2s_interface::activateWIFI::Response &res)
      {
        res.eqptId = 8;
        return true;
      }

      bool dBridge::runActivityStatus(g2s_interface::activityStatus::Request &req,
        g2s_interface::activityStatus::Response &res)
      {
        res.eqptStatus = "Equipment id X is ON";
      }

      bool dBridge::runDeactivateEqpt(g2s_interface::deactivateEqpt::Request &req,
        g2s_interface::deactivateEqpt::Response &res)
      {
        res.success = true;
      }

      bool dBridge::getPowerStatus(g2s_interface::powerStatus::Request &req,
        g2s_interface::powerStatus::Response &res)
      {
        res.powered = true;
      }

      bool dBridge::runResetEqpt(g2s_interface::resetEqpt::Request &req,
        g2s_interface::resetEqpt::Response &res)
      {
        res.success = true;
      }

      bool dBridge::runResume_Action(g2s_interface::resume_Action::Request &req,
        g2s_interface::resume_Action::Response &res)
      {
        res.success = true;
      }

      bool dBridge::runRunASCEND_GPS(g2s_interface::runASCEND_GPS::Request &req,
        g2s_interface::runASCEND_GPS::Response &res)
      {
        res.actionId = 9;
        return true;
      }

      bool dBridge::runRunASCEND(g2s_interface::runASCEND::Request &req,
        g2s_interface::runASCEND::Response &res)
      {
        res.actionId = 10;
        return true;
      }

      bool dBridge::runCIRCLE_USBL(g2s_interface::runCIRCLE_USBL::Request &req,
        g2s_interface::runCIRCLE_USBL::Response &res)
      {
        res.actionId = 11;
        return true;
      }

      bool dBridge::runDIVE(g2s_interface::runDIVE::Request &req,
        g2s_interface::runDIVE::Response &res)
      {
        res.actionId = 12;
        return true;
      }

      bool dBridge::runFOLLOW_ROW(g2s_interface::runFOLLOW_ROW::Request &req,
        g2s_interface::runFOLLOW_ROW::Response &res)
      {
        res.actionId = 13;
        return true;
      }

      bool dBridge::runFOLLOW_STRUCTURE(g2s_interface::runFOLLOW_STRUCTURE::Request &req,
        g2s_interface::runFOLLOW_STRUCTURE::Response &res)
      {
        res.actionId = 14;
        return true;
      }

      bool dBridge::runGOTO_WAYPOINT(g2s_interface::runGOTO_WAYPOINT::Request &req,
        g2s_interface::runGOTO_WAYPOINT::Response &res)
      {
        res.actionId = 15;
        return true;
      }

      bool dBridge::runGRASP_OBJECT(g2s_interface::runGRASP_OBJECT::Request &req,
        g2s_interface::runGRASP_OBJECT::Response &res)
      {
        res.actionId = 16;
        return true;
      }

      bool dBridge::runSONAR_SCANNING(g2s_interface::runSONAR_SCANNING::Request &req,
        g2s_interface::runSONAR_SCANNING::Response &res)
      {
        res.actionId = 17;
        return true;
      }

      bool dBridge::runTRACK_TARGET(g2s_interface::runTRACK_TARGET::Request &req,
        g2s_interface::runTRACK_TARGET::Response &res)
      {
        res.actionId = 18;
        return true;
      }

      bool dBridge::runVIDEO_ACQUISITION(g2s_interface::runVIDEO_ACQUISITION::Request &req,
        g2s_interface::runVIDEO_ACQUISITION::Response &res)
      {
        res.actionId = 19;
        return true;
      }

      bool dBridge::runWAIT(g2s_interface::runWAIT::Request &req,
        g2s_interface::runWAIT::Response &res)
      {
        res.actionId = 20;
        return true;
      }

      bool dBridge::stopEqptAction(g2s_interface::stopEqptAction::Request &req,
        g2s_interface::stopEqptAction::Response &res)
      {
        res.success = true;
        return true;
      }

      bool dBridge::suspend_Action(g2s_interface::suspend_Action::Request &req,
        g2s_interface::suspend_Action::Response &res)
      {
        res.success = true;
        return true;
      }

     void dBridge::stop(void)
     {

     }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, nodeName, ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  //ROS_INFO("[%s] Here I am ...",nodeName.c_str());
  //rsilauv::Bridge bridge(nh,nodeName,serverAddr,serverPort,vehicleName);
  drsilauv::dBridge dummyBridge(nh,nodeName);

  ros::spin();
  return 0;
}
