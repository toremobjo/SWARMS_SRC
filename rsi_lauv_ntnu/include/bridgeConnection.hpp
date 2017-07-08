#ifndef RSI_LAUV_NTNU_BRIDGE_HPP_INCLUDED_
#define RSI_LAUV_NTNU_BRIDGE_HPP_INCLUDED_

// ROS headers
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>


//#include "rsi_lauv_ntnu/testMsgRsiLauv.h"
//#include "rsi_lauv_ntnu/testSrvRsiLauv.h"
#include "rsi_lauv_ntnu/testStationKeeping.h"

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

#include <map>

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Local headers
#include <TcpLink.hpp>
#include <utilFctn.hpp>
#include <imcIdDefs.h>
#include <actionUtil.hpp>
//#include <actionBridge.hpp>
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

    std::map<std::string, ros::Publisher> pubMap_;
    std::map<std::string, ros::ServiceServer> serMap_;

    ros::ServiceServer ser7_;
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

    // ACTIONS, action status, goto waypoint, stationkeeping, loiter
    bool  getActionStatus(g2s_interface::actionStatus::Request &req,
      g2s_interface::actionStatus::Response &res);

    //Move to action rsi_rgi - function
    bool runGotoWaypoint(g2s_interface::runGOTO_WAYPOINT::Request &req,
      g2s_interface::runGOTO_WAYPOINT::Response &res);

    bool runStationKeeping(rsi_lauv_ntnu::testStationKeeping::Request &req,
    rsi_lauv_ntnu::testStationKeeping::Response &res);

    //TODO: Can we use a template here
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

    void sendToTcpServer(const DUNE::IMC::StationKeeping &msg)
    {
      tcp_client_->write(&msg);
      ROS_INFO("[%s] Sent to TCP Server (StationKeeping)",nodeName_.c_str());
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
    void start(void);
    // Copy/Past form OMST
    void stop(void);
  };
}

#endif
