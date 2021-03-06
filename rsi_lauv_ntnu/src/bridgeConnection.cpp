// ROS headers
#include <ros/ros.h>

// Local headers
#include <bridgeConnection.hpp>
#include <utilFctn.hpp>
#include <actionBridge.hpp>

static std::string nodeName = "bridgeLauvNode";

namespace rsilauv{
  Bridge::Bridge(ros::NodeHandle& node_handle,
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
    pub6_ = nh_.advertise<rsi_lauv_ntnu::robotPosSimple>("robotPosSimple",1000);
    // Services
    //ser1_ = nh_.advertiseService("testSrvRsiLauv_0",&Bridge::runTestSrvRsiLauv,this);
    ser2_ = nh_.advertiseService("powerStatus",&Bridge::runPowerStatus,this);
    ser3_ = nh_.advertiseService("runGOTO_WAYPOINT",&Bridge::runGotoWaypoint,this);
    ser4_ = nh_.advertiseService("abort_Action",&Bridge::runAbortAction,this);
    ser5_ = nh_.advertiseService("stop_Action",&Bridge::runStopAction,this);
    ser6_ = nh_.advertiseService("actionStatus",&Bridge::getActionStatus,this);
    ser7_ = nh_.advertiseService("stationkeepingHere",&Bridge::runStationKeeping,this);
    ser8_ = nh_.advertiseService("GotoSimple", &Bridge::runGotoSimple,this);
    // Timer
    timer1_ = nh_.createTimer(ros::Duration(1),&Bridge::messageOut,this);

    // EnvironmentData - default values
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

    start();
    waitConnectionDetermination();

    cpt_ = 0;
  }

  bool Bridge::waitConnectionDetermination(void)
  {
    ros::Rate loopRate(1);
    do
    {
      loopRate.sleep();
    }
    while (!isConnectedDetermined());
    return true;
  }

  void Bridge::messageOut(const ros::TimerEvent& event)
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
    if (false)
    {
      //TODO Gerer cette info de connection !!!
      ROS_INFO_STREAM("Info: " << int(tcp_client_->isConnected()));
    }
  }

  bool Bridge::runAbortAction(g2s_interface::abort_Action::Request &req,
    g2s_interface::abort_Action::Response &res)
  {
    if (!isConnectedDetermined()){
      res.success = false;
      return false;
      }
    if (req.actionId!=0 && req.actionId!=(action_id_-1))
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

  bool Bridge::runStopAction(g2s_interface::abort_Action::Request &req,
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
    // TODO: manage wrong action ID
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

  bool  Bridge::getActionStatus(g2s_interface::actionStatus::Request &req,
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
        oss << "EXECUTING";
        res.actionStatus = oss.str();
        res.progress = PCState->plan_progress;

      } else if (planState == 1)
      {
        res.actionStatus = "FINISHED";
      } else {
        res.actionStatus = "NONE";
      }
    } else if (req.actionId > (action_id_-1))
    {
      res.actionStatus = "NOT STARTED";
    } else {
      res.actionStatus = "FINISHED";
    }
    ROS_INFO("Plan control state: %d ", planState);
    return true;
  }

  bool Bridge::runGotoSimple(rsi_lauv_ntnu::runGotoSimple::Request &req,
    rsi_lauv_ntnu::runGotoSimple::Response &res)
  {
    if (!isConnectedDetermined())
      return false;

    if (!isServiceMode())
      return false;

    ROS_INFO("[%s] runGotoWaypoint Simple",nodeName_.c_str());

    plan_state_id_.clear();
    plan_id_ = "swarms simple goto";
    //TODO: fix simple goto

    /*DUNE::IMC::PlanControl pc;
    //pc = actionUtil::actionUtilMakePCGoto(req,res);
    std::string plan_id_ = "swarms";
    float desiredSpeed;*/

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
    desiredPoint = req.pos;
    //float deltaLat = desiredPoint.y/6386651.041660708; // divided by meters per radian latitude in Trondheim
    //float deltaLon = desiredPoint.x/2862544.348782668; // divided by meters per radian longditude in Trondheim

    desiredSpeed = 1.6;

    // Goto
    //manGoto.lat = 1.1072639824284860 + deltaLat; //todo: change to proper zero-point in time
    //manGoto.lon = 0.1806556449351842 + deltaLon;
    manGoto.lat = desiredPoint.x; //1.10724680839 + deltaLat; //todo: change to proper zero-point in time 1.10725186984
    manGoto.lon = desiredPoint.y; // 0.18063016312 + deltaLon;
    manGoto.z = desiredPoint.z;
    manGoto.z_units = DUNE::IMC::Z_DEPTH;
    //manGoto.yaw = 3.14; // todo: implement desired attitude at end of goto
    manGoto.speed = desiredSpeed;
    manGoto.speed_units = DUNE::IMC::SUNITS_METERS_PS;
    manGoto.timeout = 1000;
    manGoto.yaw = 0.0;
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

    ROS_INFO("Running actin number: %d",action_id_);

    res.actionId = action_id_;
    action_id_++;
    return true;
  }

  bool Bridge::runGotoWaypoint(g2s_interface::runGOTO_WAYPOINT::Request &req,
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
    pc = actionUtil::actionUtilMakePCGoto(req,res);
    sendToTcpServer(pc);

    Action currentAction;
    currentAction.requestTime   = ros::Time::now();
    currentAction.actionNumber  = action_id_;
    currentAction.actionPlan    = pc;
    actionArray.push_back(currentAction);

    ROS_INFO("Running actin number: %d",action_id_);
    action_id_++;
    res.actionId = currentAction.actionNumber;

    return true;
  }

  bool Bridge::runStationKeeping(rsi_lauv_ntnu::testStationKeeping::Request &req,
  rsi_lauv_ntnu::testStationKeeping::Response &res){

    if (!isConnectedDetermined())
      return false;

    if (!isServiceMode())
      return false;

    std::string plan_id_ = "swarms";
    float desiredSpeed;

    DUNE::IMC::PlanControl pc;
    pc.plan_id = plan_id_;
    pc.op = DUNE::IMC::PlanControl::PC_START;
    pc.type = DUNE::IMC::PlanControl::PC_REQUEST;
    pc.request_id = 1000;

    DUNE::IMC::StationKeeping sk;
    DUNE::IMC::PlanManeuver pm;
    DUNE::IMC::PlanSpecification ps;

    //calculate from relative position(NED) in relation to Fixed point.
    geometry_msgs::Point desiredPoint;
    desiredSpeed = 1.6;

    // Goto
    sk.lat = mySitu_.lat;//1.1072639824284860;// + deltaLat; //todo: change to proper zero-point in time
    sk.lon = mySitu_.lon;//0.1806556449351842;// + deltaLon;
    sk.z = mySitu_.z; //desiredPoint.z;
    sk.z_units = DUNE::IMC::Z_DEPTH;
    sk.radius = 20;
    //manGoto.yaw = 3.14; // todo: implement desired attitude at end of goto
    sk.speed = desiredSpeed;
    sk.speed_units = DUNE::IMC::SUNITS_METERS_PS;

    // Maneuver
    pm.maneuver_id = "123";
    pm.data.set(sk);
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
    res.actionId = currentAction.actionNumber;

    return true;
  }


  bool Bridge::runPowerStatus(g2s_interface::powerStatus::Request &req,
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


  void Bridge::messageIn(const DUNE::IMC::Message* msg)
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
          //mySitu_.lat   = ppp->lat;
          //mySitu_.lon   = ppp->lon;
          mySitu_.latlin= ppp->lat;
          mySitu_.lonlin= ppp->lon;
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

          //Calculate the estimated longditude and latitude from lineariziation
          //around the local origin of DUNE latitude and lingditude
          //This will most likely be set to the origin point.
          mySitu_.lat = mySitu_.latlin + mySitu_.x/6371000.0;
          mySitu_.lon = mySitu_.lonlin + mySitu_.y/(6371000.0*cos(mySitu_.latlin));

          //ROS_INFO("My estimated latitude: %.20f xpos: %.20f",mySitu_.lat,mySitu_.x);
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

          geometry_msgs::Point simplePos;
          simplePos.x = mySitu_.lat;
          simplePos.y = mySitu_.lon;
          simplePos.z = mySitu_.z;
          rsi_lauv_ntnu::robotPosSimple posmsg;
          posmsg.pos = simplePos;
          posmsg.header.stamp = ros::Time::now();
          pub6_.publish(posmsg);
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
            //TODO: SEGFAULT DONE
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

        case IMC_ID_PRESSURE :
        {
          const DUNE::IMC::Pressure* ppp = static_cast<const DUNE::IMC::Pressure*>(msg);
          myWater_.pressure = ppp->value;
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
          //ROS_INFO("Recieved Plan Control State IMC message");
          //TODO: SEGFAULT

          if (PCStateCounter>2)
          {
            lastLastPCState = lastPCState;
            lastPCState = PCState->state;
          }
          PCState = static_cast<const DUNE::IMC::PlanControlState*>(msg);
           if (PCStateCounter > 2)
            {
              if (PCState->state == 1  && lastLastPCState == 3 /*PCState->state*/)
              //TODO: SEGFAULT
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

        case IMC_ID_STATIONKEEPING:
        {
          //ROS_INFO("Recieved stationkeeping IMC message");
        }

        case IMC_ID_PLANCONTROL :
        {
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

  bool Bridge::isConnectedDetermined(void)
  {
    if (tcp_client_->isConnected())
    {
      if (isVehicleIdDetermined())
      {
        //ROS_INFO("Vehicle connected.");
        return true;
      }
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

  bool Bridge::isServiceMode(void)
  {
    if (!vehicleService_)
    {
      ROS_WARN("[%s] Vehicle is not in service mode.",nodeName_.c_str());
    }
    return vehicleService_;
  }

  bool Bridge::isFromVehicle(int IdSource)
  {
    return (vehicleId_==IdSource);
  }

  bool Bridge::isVehicleIdDetermined(void)
  {
    return (vehicleId_>0);
  }

  void Bridge::determineVehicleId(const DUNE::IMC::Announce* msg)
  {
    if (msg->sys_name==vehicleName_)
      vehicleId_ = msg->getSource();
  }

  void Bridge::start(void)
  {
    stop();
    tcp_client_ = new ros_imc_broker::TcpLink(boost::bind(&Bridge::messageIn, this, _1));
    tcp_client_->setServer(serverAddr_, serverPort_);
    tcp_client_thread_ = new boost::thread(boost::ref(*tcp_client_));
  }

  void Bridge::stop(void)
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
  return 0;
}
