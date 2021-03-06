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
    Bridge(ros::NodeHandle& node_handle,
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
      // Messages
      pub1_ = nh_.advertise<rsi_lauv_ntnu::testMsgRsiLauv>("testMsgRsiLauv_0",1000);
      pub2_ = nh_.advertise<g2s_interface::robotStatus>("robotStatus",1000);
      pub3_ = nh_.advertise<g2s_interface::endOfAction>("endOfAction",1000);
      pub4_ = nh_.advertise<g2s_interface::environmentData>("environmentData",1000);
      pub5_ = nh_.advertise<g2s_interface::robotSituation>("robotSituation",1000);
      // Services
      ser1_ = nh_.advertiseService("testSrvRsiLauv_0",&Bridge::runTestSrvRsiLauv,this);
      ser2_ = nh_.advertiseService("powerStatus",&Bridge::runPowerStatus,this);
      ser3_ = nh_.advertiseService("runGOTO_WAYPOINT",&Bridge::runGotoWaypoint,this);
      ser4_ = nh_.advertiseService("abort_Action",&Bridge::runAbortAction,this);
      ser5_ = nh_.advertiseService("stop_Action",&Bridge::runStopAction,this);
      ser6_ = nh_.advertiseService("actionStatus",&Bridge::getActionStatus,this);
      // Timer
      timer1_ = nh_.createTimer(ros::Duration(1),&Bridge::messageOut,this);



      // "EnvironmentData"
      myWater_.temperature = -99999;
      myWater_.salinity = -99999;
      myWater_.conductivity = -99999;
      myWater_.pressure = -99999;
      myWater_.soundSpeed = -99999;
      myWater_.turbidity = -99999;
      myWater_.ph = -99999;
      myWater_.levelH2S = -99999;
      myWater_.levelDVL = -99999;

      // RobotSituation
      mySitu_.x = 0;
      mySitu_.y = 0;
      mySitu_.z = 0;
      mySitu_.sequence = 1;

      PCStateCounter = 0;
      lastPCState = 1;
      lastLastPCState = 1;
      //

      start();
      waitConnectionDetermination();

      cpt_ = 0;
    }
    ~Bridge(void)
    {
      stop();
    }

    bool waitConnectionDetermination(void)
    {
      ros::Rate loopRate(0.2);
      do
      {
        loopRate.sleep();
      }
      while (!isConnectedDetermined());
      return true;
    }

    void messageOut(const ros::TimerEvent& event)
    {
      if (!isConnectedDetermined())
        return;

      if (true)
      {
        cpt_ ++;
        rsi_lauv_ntnu::testMsgRsiLauv msg_n;
        msg_n.txt1 = "test1";
        msg_n.txt2 = "test2";
        msg_n.val1 = cpt_;
        msg_n.val2 = cpt_*-1;
        pub1_.publish(msg_n);
        //rostopic echo /testMsgRsiLauv
      }
      if (true)
      {
        g2s_interface::robotStatus msg_r;
        msg_r.robotBatteryLevel = cpt_;
        msg_r.robotAutonomy = 50;
        pub2_.publish(msg_r);
        //rostopic echo /robotStatus
      }
      if (true)
      {
        g2s_interface::environmentData msg_ed;
        msg_ed.waterData.wTemperature = myWater_.temperature;
        msg_ed.waterData.wSalinity = myWater_.salinity;
        msg_ed.waterData.wPressure = myWater_.pressure;
        msg_ed.waterData.wSoundSpeed = myWater_.soundSpeed;
        msg_ed.waterData.wTurbidity = myWater_.turbidity;
        msg_ed.waterData.wPH = myWater_.ph;
        msg_ed.waterData.wLevelH2S = myWater_.levelH2S;
        msg_ed.waterData.wLevelDVL = myWater_.levelDVL;
        pub4_.publish(msg_ed);
      }
      if (true)
      {
        /*g2s_interface::robotSituation msg_rsit;
        msg_rsit.robotPose.position.x = mySitu_.x;
        msg_rsit.robotPose.position.y = mySitu_.y;
        msg_rsit.robotPose.position.z = mySitu_.z;
        pub5_.publish(msg_rsit);*/
        //rostopic echo /robotSituation
      }
      if (false)
      {
        //TODO Gerer cette info de connection !!!
        ROS_INFO_STREAM("Info: " << int(tcp_client_->isConnected()));
      }
    }

  private:


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
      g2s_interface::abort_Action::Response &res)
    {
      if (!isConnectedDetermined()){
        res.success = false;
        return false;
      }

      //ROS_INFO("[%s] val %d, val2 %d",nodeName_.c_str(),action_id_,req.actionId);

      if (req.actionId!=0 && req.actionId!=action_id_)
      {
        ROS_WARN("[%s] Incorrect actionId",nodeName_.c_str());
        res.success = false;
        return false;
      }

      ROS_INFO("[%s] runAbortAction",nodeName_.c_str());

      DUNE::IMC::Abort ab;
      ab.setDestination(vehicleId_);
      sendToTcpServer(ab);

      //ABORT EVERYTHING - what happens if the camera/sonar is activated, but the rest is aborted?

      ROS_INFO("plan control state: %d ",res.success);
      res.success = true;
      return true;

    }

    bool runStopAction(g2s_interface::abort_Action::Request &req,
      g2s_interface::abort_Action::Response &res)
    {
      if (!isConnectedDetermined()){
        res.success = false;
        return false;
      }

      if (req.actionId!=0 && req.actionId!=action_id_)
      {
        ROS_WARN("[%s] Incorrect actionId",nodeName_.c_str());
        res.success = false;
        return false;
      }

      ROS_INFO("[%s] runStopCurrentAction",nodeName_.c_str());

      DUNE::IMC::PlanControl pc;
      pc.plan_id = "swarms";
      pc.op = DUNE::IMC::PlanControl::PC_STOP;
      pc.type = DUNE::IMC::PlanControl::PC_REQUEST;
      pc.request_id = 1000;
      sendToTcpServer(pc);

      Action currentAction;
      currentAction.requestTime   = ros::Time::now();
      currentAction.actionNumber  = action_id_;
      currentAction.actionPlan    = pc;
      actionArray.push_back(currentAction);
      action_id_++;

      res.success = true;
      return true;

    }



    bool  getActionStatus(g2s_interface::actionStatus::Request &req,
      g2s_interface::actionStatus::Response &res)
    {
      if (!isConnectedDetermined())
        return false;

      uint8_t planState =  PCState->state;

      if (req.actionId == (action_id_-1))
      {
        if (planState == 2)
        {
          res.actionStatus = "INITIALIZING";
        } else if (planState == 3)
        {
          std::ostringstream oss;
          oss << "EXECUTING, progress: " << PCState->plan_progress << " %%";
          res.actionStatus = oss.str();

        } else if (planState == 1)
        {
          res.actionStatus = "FINISHED";
        } else {
          res.actionStatus = "NONE";
        }
      } else if (req.actionId > (action_id_-1))
      {
        res.actionStatus = "NOT STARTED";
      }/*else if (req.actionId == (action_id_-2))
      {
        int size = actionArray.size();
        if (PCState->last_outcome == 1 )
            {
              //int size = actionArray.size();
              actionArray[size-2].success = true;
              ROS_INFO("Last action was successful");
              res.actionStatus = "FINISHED SUCCESSFULLY";
            } else if (PCState->last_outcome == 2)
            {

              actionArray[size-2].success = false;
              ROS_INFO("Last action failed");
              res.actionStatus = "FINISHED UNSUCCESSFULLY";
            }

      }*/ else {
        res.actionStatus = "FINISHED";
      }
      ROS_INFO("Plan control state: %d ", planState);
      return true;
    }

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
      g2s_interface::powerStatus::Response &res)
    {
      if (!isConnectedDetermined())
        return false;

      ROS_INFO("[%s] Power Status Service of Equipment Id: %d",nodeName_.c_str(),req.eqptId);
      if (req.eqptId==33)
        res.powered = true;
      else
        res.powered = false;
      return true;
    }

    void messageIn(const DUNE::IMC::Message* msg)
    {
      if (!isVehicleIdDetermined())
      {
        if (msg->getId()==IMC_ID_ANNOUNCE)
          determineVehicleId(static_cast<const DUNE::IMC::Announce*>(msg));
        return;
      }
      if (isFromVehicle(msg->getSource()))
      {
        switch(msg->getId()){

          case IMC_ID_ESTIMATEDSTATE :
          {
            const DUNE::IMC::EstimatedState* ppp = static_cast<const DUNE::IMC::EstimatedState*>(msg);
            mySitu_.x     = ppp->x;
            mySitu_.y     = ppp->y;
            mySitu_.z     = ppp->z;
            mySitu_.lat   = ppp->lat;
            mySitu_.lon   = ppp->lon;
            mySitu_.height= ppp->height;
            mySitu_.phi   = ppp->phi;
            mySitu_.theta = ppp->theta;
            mySitu_.psi   = ppp->psi;
            mySitu_.u     = ppp->u;
            mySitu_.v     = ppp->v;
            mySitu_.w     = ppp->w;
            mySitu_.vx    = ppp->vx;
            mySitu_.vy    = ppp->vy;
            mySitu_.vz    = ppp->vz;
            mySitu_.p     = ppp->p;
            mySitu_.q     = ppp->q;
            mySitu_.r     = ppp->r;
            mySitu_.depth = ppp->depth;
            mySitu_.altitude = ppp->alt;

            // generate g2s robot situation message
            g2s_interface::robotSituation situ;
            //Point posiiton
            situ.robotPose.position.x   = mySitu_.x;
            situ.robotPose.position.y   = mySitu_.y;
            situ.robotPose.position.z   = mySitu_.z;

            //Quaternion orientation
            double t0 = std::cos(mySitu_.psi * 0.5);
          	double t1 = std::sin(mySitu_.psi * 0.5);
          	double t2 = std::cos(mySitu_.phi * 0.5);
          	double t3 = std::sin(mySitu_.phi * 0.5);
          	double t4 = std::cos(mySitu_.theta * 0.5);
          	double t5 = std::sin(mySitu_.theta * 0.5);
          	situ.robotPose.orientation.w = t0 * t2 * t4 + t1 * t3 * t5;
          	situ.robotPose.orientation.x = t0 * t3 * t4 - t1 * t2 * t5;
          	situ.robotPose.orientation.y = t0 * t2 * t5 + t1 * t3 * t4;
          	situ.robotPose.orientation.z = t1 * t2 * t4 - t0 * t3 * t5;

            //Body forward speed u
            situ.robotSpeed = mySitu_.u;

            // Header: stamp, name and sequence number
            situ.header.stamp     = ros::Time::now();
            situ.header.frame_id  = "NTNU_LAUV_1";
            situ.header.seq       =  mySitu_.sequence;
            mySitu_.sequence++;

            // Altitude above the sea floor
            situ.robotAltitude = mySitu_.altitude;

            //Publish situation message
            pub5_.publish(situ);
            break;
          }

          case IMC_ID_ANNOUNCE :
          {
            //TBD
            //ROS_INFO("Announce");
            break;
          }

          case IMC_ID_VEHICLESTATE :
          {
            const DUNE::IMC::VehicleState* ppp = static_cast<const DUNE::IMC::VehicleState*>(msg);
            vehicleService_ = ppp->op_mode==DUNE::IMC::VehicleState::VS_SERVICE;
            //ROS_INFO("VehicleState");
            break;
          }

          case IMC_ID_HEARTBEAT :
          {
            //TBD
            //ROS_INFO("heartbeat");
            break;
          }

          case IMC_ID_FUELLEVEL :
          {
            const DUNE::IMC::FuelLevel* ppp = static_cast<const DUNE::IMC::FuelLevel*>(msg);
            // trace("Operation modes are: %s\nPercentage is %.2f\nConfidence level is %.2f\n",
            //      m_fuel.opmodes.c_str(), m_fuel.value, m_fuel.confidence);
            ROS_INFO_THROTTLE(60,"MSG_ID: %d --> FuelLevel=%.2f (CI=%.2f) [%s]", int(msg->getId()),ppp->value,ppp->confidence,ppp->opmodes.c_str());
            break;
          }

          case IMC_ID_ENTITYACTIVATIONSTATE :
          {
            //TBD
            //ROS_INFO("EntityActivationState");
            break;
          }

          case IMC_ID_ENTITYLIST :
          {
            if (!flagEntity_)
            {
              //ROS_INFO("MSG_ID: %d --> EntityList",int(msg->getId()));
              const DUNE::IMC::EntityList* ppp = static_cast<const DUNE::IMC::EntityList*>(msg);
              //ROS_INFO("MSG_ID: %d --> EntityList: %s",int(msg->getId()),ppp->list.c_str());
              //TODO: Parse EntityList
              flagEntity_ = true;
              idEntityCTD1_ = 39;
              idEntityCTD2_ = 47;
            }
            break;
          }

          /*case IMC_ID_SAVEENTITYPARAMETERS :
          {
            break;
          }*/

          case IMC_ID_CONDUCTIVITY :
          {
           const DUNE::IMC::Conductivity* ppp = static_cast<const DUNE::IMC::Conductivity*>(msg);
           myWater_.conductivity = ppp->value;
           break;
          }

          case IMC_ID_SALINITY :
          {
            const DUNE::IMC::Salinity* ppp = static_cast<const DUNE::IMC::Salinity*>(msg);
            myWater_.salinity = ppp->value;
            break;
          }

          case IMC_ID_TEMPERATURE :
          {
            const DUNE::IMC::Temperature* ppp = static_cast<const DUNE::IMC::Temperature*>(msg);
            myWater_.temperature = ppp->value;
            break;
          }

          case IMC_ID_SOUNDSPEED :
          {
          const DUNE::IMC::SoundSpeed* ppp = static_cast<const DUNE::IMC::SoundSpeed*>(msg);
          myWater_.soundSpeed = ppp->value;
          break;
          }

          case IMC_ID_DEPTH:
          {
            const DUNE::IMC::Depth* ppp = static_cast<const DUNE::IMC::Depth*>(msg);
            break;
          }

          case IMC_ID_PLANCONTROLSTATE :
          {
            if (PCStateCounter>2)
            {
              lastLastPCState = lastPCState;
              lastPCState = PCState->state;
            }

            PCState = static_cast<const DUNE::IMC::PlanControlState*>(msg);


             if (PCStateCounter > 2)
              {
                //ROS_INFO("Plan Control states: %d  and %d ",PCState->state, lastLastPCState);

                if (PCState->state == 1  && lastLastPCState == 3 /*PCState->state*/)
                {
                  int size = actionArray.size();
                  actionArray[size-1].success = true;

                  g2s_interface::endOfAction msg;
                  msg.actionId = action_id_-1;
                  msg.endCode = 1;
                  pub3_.publish(msg);
                }
              }

            PCStateCounter++;


            break;
          }

          case IMC_ID_PLANCONTROL :
          {/*
            PC = static_cast<const DUNE::IMC::PlanControl*>(msg);
            int size = actionArray.size();
            if (PC->type == 1 )
            {
              //int size = actionArray.size();
              actionArray[size-1].success = true;
              ROS_INFO("Last action was successful");
            } else if (PC->type == 2)
            {

              actionArray[size-1].success = false;
              ROS_INFO("Last action failed");
            }

            ROS_INFO("new pc message: %d",actionArray[size-1].success);*/


            break;
          }

          default :
          {
            ROS_INFO("Unknown message type, please identify message.");
            ROS_INFO("MSG_ID: %d (?)", int(msg->getId()));
          }
        }
      }
    }

    bool isConnectedDetermined(void)
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

    bool isServiceMode(void)
    {
      if (!vehicleService_)
      {
        ROS_WARN("[%s] Vehicle is not in service mode.",nodeName_.c_str());
      }
      return vehicleService_;
    }

    bool isFromVehicle(int IdSource)
    {
      return (vehicleId_==IdSource);
    }

    bool isVehicleIdDetermined(void)
    {
      return (vehicleId_>0);
    }

    void determineVehicleId(const DUNE::IMC::Announce* msg)
    {
      if (msg->sys_name==vehicleName_)
        vehicleId_ = msg->getSource();
    }

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
