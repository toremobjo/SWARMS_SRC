#ifndef RSI_LAUV_NTNU_BRIDGE_HPP_INCLUDED_
#define RSI_LAUV_NTNU_BRIDGE_HPP_INCLUDED_

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
// #include <stateLauv.hpp>

namespace rsilauv
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
    float lat, lon, height; // Latitudinal, longditudinal and height position estimate.
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
    DUNE::IMC::PlanControl actionPlan;
    bool success;
  };
  class Bridge
  {
  public:
    // Constructor and Destructor
    Bridge(ros::NodeHandle& node_handle,
      std::string& nodeName,
      std::string& serverAddr,
      std::string& serverPort,
      std::string& vehicleName);
    ~Bridge(void){stop();}

    //Connection determinantion
    bool waitConnectionDetermination(void);
    void messageOut(const ros::TimerEvent& event);

  private:
    //TODO Fix this mess
    ros::NodeHandle& nh_;
    std::string& nodeName_;

    std::string& serverAddr_;
    std::string& serverPort_;
    ros_imc_broker::TcpLink* tcp_client_;         // TCP client to DUNE's server
    boost::thread* tcp_client_thread_;            // TCP client thread

    std::string& vehicleName_;
    int vehicleId_;
    bool vehicleService_;
    int nbActions_;

    ros::Publisher pub1_;
    ros::Publisher pub2_;
    ros::Publisher pub3_;
    ros::Publisher pub4_;
    ros::Publisher pub5_;
    ros::ServiceServer ser1_;
    ros::ServiceServer ser2_;
    ros::ServiceServer ser3_;
    ros::ServiceServer ser4_;
    ros::ServiceServer ser5_;
    ros::ServiceServer ser6_;
    ros::Timer timer1_;

    //! Desired plan id.
    std::string plan_id_;
    int action_id_;
    //! Executing plan id.
    std::string plan_state_id_;

    Water myWater_;
    Situation mySitu_;

    //Action Identification and storage
    std::vector<Action> actionArray;


    int cpt_;
    bool flagEntity_;
    int idEntityCTD1_;
    int idEntityCTD2_;
    bool flagInitPlanOutCome_;

    int lastState_;

    int PCStateCounter;

    utilfctn::TicToc chrono1_;
    const DUNE::IMC::PlanControlState* PCState;
    int lastPCState;
    int lastLastPCState;
    const DUNE::IMC::PlanControl* PC;

    float desiredSpeed;

    //Abort spesific  action, do nothing after.
    bool runAbortAction(g2s_interface::abort_Action::Request &req,
      g2s_interface::abort_Action::Response &res);

    bool runStopAction(g2s_interface::abort_Action::Request &req,
      g2s_interface::abort_Action::Response &res);


    bool  getActionStatus(g2s_interface::actionStatus::Request &req,
      g2s_interface::actionStatus::Response &res);

    //Move to action rsi_rgi - function
    bool runGotoWaypoint(g2s_interface::runGOTO_WAYPOINT::Request &req,
      g2s_interface::runGOTO_WAYPOINT::Response &res)
    {
      if (!isConnectedDetermined())
        return false;

      if (!isServiceMode())
        return false;

      ROS_INFO("[%s] runGotoWaypoint",nodeName_.c_str());

      //Todo: edit such that it fits service declaration, current declaration(26.06.2017) has
      // some redundancies and will probably be changed.

      plan_state_id_.clear();
      plan_id_ = "swarms";

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
      sendToTcpServer(pc);

      Action currentAction;
      currentAction.requestTime   = ros::Time::now();
      currentAction.actionNumber  = action_id_;
      currentAction.actionPlan    = pc;
      actionArray.push_back(currentAction);

      //ROS_INFO("Running actin number: %d",action_id_);
      action_id_++;

      nbActions_ ++;
      res.actionId = currentAction.actionNumber;

      return true;
    }


    // Move or remove

    bool runTestSrvRsiLauv(rsi_lauv_ntnu::testSrvRsiLauv::Request &req,
      rsi_lauv_ntnu::testSrvRsiLauv::Response &res)
    {
      if (!isConnectedDetermined())
      {
        return false;
      }
      if (req.ind1==40)
      {
        DUNE::IMC::EntityParameter p;
        p.name = "Active";
        p.value = "false";
        DUNE::IMC::SetEntityParameters msg;
        msg.name = "Camera";
        msg.params.push_back(p);
        sendToTcpServer(msg);
      }
      // Message ID: 804 (?)
      if (req.ind1==30)
      {
        DUNE::IMC::EntityParameter p;
        p.name = "Active";
        p.value = "true";
        DUNE::IMC::SetEntityParameters msg;
        msg.name = "Camera";
        msg.params.push_back(p);
        sendToTcpServer(msg);
      }

      if (req.ind1==20)
      {
        DUNE::IMC::QueryEntityActivationState qeas;
        // qeas.setDestinationEntity(resolveEntity("Formation Control"));
        qeas.setDestinationEntity(req.ind2);
        sendToTcpServer(qeas);
        ROS_INFO("[%s] Voila ...",nodeName_.c_str());
        return true;
      }

      if (req.ind1==10)
      {
        DUNE::IMC::QueryEntityParameters qep;
        qep.name = "Sidescan";
        // qep.name = "Path Control"; // "CTD Simulator"; "Sidescan"
        sendToTcpServer(qep);
        ROS_INFO("[%s] DONEEEEEEE",nodeName_.c_str());
        return true;
      }

      // Abort <=> Stop a plan
      if (req.ind1==0)
      {
        DUNE::IMC::PlanControl pc;
        pc.plan_id = plan_id_;
        pc.op = DUNE::IMC::PlanControl::PC_STOP;
        pc.type = DUNE::IMC::PlanControl::PC_REQUEST;
        pc.request_id = 1000;
        sendToTcpServer(pc);
        res.val1 = 1;
        res.txt1 = "Service done";
        return true;
      }

      if (!isServiceMode())
        return false;

      plan_state_id_.clear();
      plan_id_ = "swarms";

      DUNE::IMC::PlanControl pc;
      pc.plan_id = plan_id_;
      pc.op = DUNE::IMC::PlanControl::PC_START;
      pc.type = DUNE::IMC::PlanControl::PC_REQUEST;
      pc.request_id = 1000;

      DUNE::IMC::Goto manGoto, manGoto1, manGoto2 ;
      DUNE::IMC::Loiter manLoiter;
      DUNE::IMC::PlanManeuver pm, pm1, pm2;
      DUNE::IMC::PlanSpecification ps;

      if (req.ind1==1)
      {
        // Goto
        manGoto.lat = 1.1072639824284860;
        manGoto.lon = 0.1806556449351842;
        manGoto.z = 0;
        manGoto.z_units = DUNE::IMC::Z_DEPTH;
        manGoto.speed = 1.5;
        manGoto.speed_units = DUNE::IMC::SUNITS_METERS_PS;
        manGoto.timeout = 20.0;
        // Maneuver
        pm.maneuver_id = "1";
        pm.data.set(manGoto);
        // Specification
        ps.plan_id = pc.plan_id;
        ps.start_man_id = pm.maneuver_id;
        ps.maneuvers.push_back(pm);
      }
      else if (req.ind1==2)
      {
        // Goto
        manGoto.lat = 1.1072572803641585;
        manGoto.lon = 0.1806374062445009;
        manGoto.z = 0;
        manGoto.z_units = DUNE::IMC::Z_DEPTH;
        manGoto.speed = 1.5;
        manGoto.speed_units = DUNE::IMC::SUNITS_METERS_PS;
        manGoto.timeout = 20.0;
        // Maneuver
        pm.maneuver_id = "1";
        pm.data.set(manGoto);
        // Specification
        ps.plan_id = pc.plan_id;
        ps.start_man_id = pm.maneuver_id;
        ps.maneuvers.push_back(pm);
      }
      else if (req.ind1==3)
      {
        // Goto
        manGoto1.lat = 1.1072639824284860;
        manGoto1.lon = 0.1806556449351842;
        manGoto1.z = 0;
        manGoto1.z_units = DUNE::IMC::Z_DEPTH;
        manGoto1.speed = 1.5;
        manGoto1.speed_units = DUNE::IMC::SUNITS_METERS_PS;
        manGoto1.timeout = 20.0;
        manGoto2.lat = 1.1072572803641585;
        manGoto2.lon = 0.1806374062445009;
        manGoto2.z = 0;
        manGoto2.z_units = DUNE::IMC::Z_DEPTH;
        manGoto2.speed = 1.5;
        manGoto2.speed_units = DUNE::IMC::SUNITS_METERS_PS;
        manGoto2.timeout = 20.0;
        // Maneuvers
        pm1.maneuver_id = "1";
        pm1.data.set(manGoto1);
        pm2.maneuver_id = "2";
        pm2.data.set(manGoto2);
        // Transition
        DUNE::IMC::PlanTransition trans;
        trans.conditions = "ManeuverIsDone";
        trans.source_man = pm1.maneuver_id;
        trans.dest_man = pm2.maneuver_id;
        // Specification
        ps.plan_id = pc.plan_id;
        ps.start_man_id = pm1.maneuver_id;
        ps.maneuvers.push_back(pm1);
        ps.transitions.push_back(trans);
        ps.maneuvers.push_back(pm2);
      }
      else if (req.ind1==4)
      {
        // Loiter
        manLoiter.timeout = 180.0;
        manLoiter.lat = 1.1072572803641585;
        manLoiter.lon = 0.1806374062445009;
        manLoiter.z = 0;
        manLoiter.z_units = DUNE::IMC::Z_DEPTH;
        manLoiter.duration = 120;
        manLoiter.speed = 1.3;
        manLoiter.speed_units = DUNE::IMC::SUNITS_METERS_PS;
        manLoiter.type = DUNE::IMC::Loiter::LT_CIRCULAR;
        manLoiter.radius = 20;
        manLoiter.direction = DUNE::IMC::Loiter::LD_CLOCKW;
        // Maneuvers
        pm.maneuver_id = "1";
        pm.data.set(manLoiter);
        // Specification
        ps.plan_id = pc.plan_id;
        ps.start_man_id = pm.maneuver_id;
        ps.maneuvers.push_back(pm);
      }
      // PLAN CONTROL
      pc.arg.set(ps);
      sendToTcpServer(pc);

      //Action storage
      Action currentAction;
      currentAction.requestTime   = ros::Time::now();
      currentAction.actionNumber  = action_id_;
      currentAction.actionPlan    = pc;
      actionArray.push_back(currentAction);
      action_id_++;

      res.val1 = double(req.ind1);
      res.txt1 = "Service done";
      return true;
    }

    //TODO: Peut on passer par un Template
    void sendToTcpServer(const DUNE::IMC::SetEntityParameters &msg)
    {
      tcp_client_->write(&msg);
      ROS_INFO("[%s] Sent to TCP Server (SetEntityParameters)",nodeName_.c_str());
    }

    void sendToTcpServer(const DUNE::IMC::QueryEntityActivationState &msg)
    {
      tcp_client_->write(&msg);
      ROS_INFO("[%s] Sent to TCP Server (QueryActivationState)",nodeName_.c_str());
    }

    void sendToTcpServer(const DUNE::IMC::QueryEntityParameters &msg)
    {
      tcp_client_->write(&msg);
      ROS_INFO("[%s] Sent to TCP Server (QueryEntityParameters)",nodeName_.c_str());
    }

    void sendToTcpServer(const DUNE::IMC::PlanControl &msg)
    {
      tcp_client_->write(&msg);
      ROS_INFO("[%s] Sent to TCP Server (PlanControl)",nodeName_.c_str());
    }

    void sendToTcpServer(const DUNE::IMC::Abort &msg)
    {
      tcp_client_->write(&msg);
      ROS_INFO("[%s] Sent to TCP Server (Abort)",nodeName_.c_str());
    }

    bool runPowerStatus(g2s_interface::powerStatus::Request &req,
      g2s_interface::powerStatus::Response &res);

    //Collects and relays all defined/desired messages from DUNE
    void messageIn(const DUNE::IMC::Message* msg);

    // We need connection to DUNE
    bool isConnectedDetermined(void);

    //Is the lauv in servicemode? if not, we cannot perform an action
    // exept stop or abort.
    bool isServiceMode(void);

    //Find the identity of the vehicle
    bool isFromVehicle(int IdSource);

    //Is the vehicle identity/vehicle all together available?
    bool isVehicleIdDetermined(void);

    void determineVehicleId(const DUNE::IMC::Announce* msg);

    // Copy/Past form OMST
    void start(void)
    {
      stop();
      tcp_client_ = new ros_imc_broker::TcpLink(boost::bind(&Bridge::messageIn, this, _1));
      tcp_client_->setServer(serverAddr_, serverPort_);
      tcp_client_thread_ = new boost::thread(boost::ref(*tcp_client_));
    }
    // Copy/Past form OMST
    void stop(void)
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
  };
}

#endif
