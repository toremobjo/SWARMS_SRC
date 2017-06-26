#include <ros/ros.h>
#include "g2s_interface/endOfAction.h"
#include "g2s_interface/abort_Action.h"
#include "g2s_interface/runGOTO_WAYPOINT.h"
#include <std_msgs/Int32.h>
#include "rsi_lauv_ntnu/testMsgRsiLauv.h"
#include <actionDefs.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

static std::string nodeName = "fakeRgi";

class fRgi 
{
public: 
	fRgi();
	void spin();

private: 
	ros::NodeHandle nh;
	ros::Subscriber actions;

	ros::ServiceClient abortClient;
	ros::ServiceClient stopClient;
	ros::ServiceClient goto1Client;
	ros::ServiceClient goto2Client;

	void testActionsCallback(const std_msgs::Int32ConstPtr& msg);

	void runAction(int action);
	void abortCurrentMission();
	void stopCurrentAction();
	void gotoWay1();
	void gotoWay2();

	bool res;

};

fRgi::fRgi(){
	actions = nh.subscribe("testActions",100, &fRgi::testActionsCallback, this);
	abortClient = nh.serviceClient<g2s_interface::abort_Action>("abort_Action",0);
	goto1Client = nh.serviceClient<g2s_interface::runGOTO_WAYPOINT>("runGOTO_WAYPOINT",0);
	goto2Client = nh.serviceClient<g2s_interface::runGOTO_WAYPOINT>("runGOTO_WAYPOINT",0);
	stopClient 	= nh.serviceClient<g2s_interface::abort_Action>("stop_Action",0);
}

void fRgi::spin(){
	ros::Rate loop(1000);
    while(ros::ok()) {
        
        loop.sleep();
        ros::spinOnce();
    }
}

void fRgi::testActionsCallback(const std_msgs::Int32ConstPtr& msg){
	//action = msg->data;
	runAction(msg->data);

}

void fRgi::runAction(int action){
	ROS_INFO("Running action: %d ", action);
	switch(action){
		case ACTION_DEF_ABORT 	: abortCurrentMission(); break;
		case ACTION_DEF_STOP	: stopCurrentAction(); break;
		case ACTION_DEF_GOTO1	: gotoWay1(); break;
		case ACTION_DEF_GOTO2	: gotoWay2(); break;
		default 				: ROS_INFO("Unknown action command");
	}

}

void fRgi::abortCurrentMission(){
	g2s_interface::abort_Action ab;
	if (abortClient.call(ab))
	{
		ROS_INFO("Abort Successful");
	}
}

void fRgi::stopCurrentAction(){
	g2s_interface::abort_Action ab;
	if (stopClient.call(ab))
	{
		ROS_INFO("Stop Successful");
	}
}

void fRgi::gotoWay1(){
	g2s_interface::runGOTO_WAYPOINT go;
	go.request.modeId = 1;

	//specify a point
	geometry_msgs::Point desiredPos;
	desiredPos.x = 0;
	desiredPos.y = 0;
	desiredPos.z = 0;

	go.request.waypointPosition = desiredPos;


	if (goto1Client.call(go))
	{
		ROS_INFO("Go to waypoint 1, successfully sent.");
	}

}

void fRgi::gotoWay2(){
	g2s_interface::runGOTO_WAYPOINT go;
	go.request.modeId = 1;

	//specify a point
	geometry_msgs::Point desiredPos;
	desiredPos.x = 400;
	desiredPos.y = 0;
	desiredPos.z = 0;

	go.request.waypointPosition = desiredPos;


	if (goto1Client.call(go))
	{
		ROS_INFO("Go to waypoint 2, successfully sent.");
	}

}

int main (int argc, char **argv)
{
	ros::init(argc, argv, nodeName, ros::init_options::AnonymousName);
	fRgi fake;
	ROS_INFO("[%s] ... Coucou ROS ...",nodeName.c_str());
  	ros::spin();
  	return 0;
}
