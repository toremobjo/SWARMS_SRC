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
	ros::Subscriber endOfAction_;

	ros::ServiceClient abortClient;
	ros::ServiceClient stopClient;
	ros::ServiceClient goto1Client;
	ros::ServiceClient goto2Client;

	void testActionsCallback(const std_msgs::Int32ConstPtr& msg);
	void endOfActionCallback(const g2s_interface::endOfAction& msg);

	void runAction(int action);
	void abortCurrentMission();
	void stopCurrentAction();
	void gotoWay1();
	void gotoWay2();
	void gotoWay3();
	void gotoWay4();
	void executePlan1();

	bool res;
	bool endOfActionBool;
	bool isRunningPath;

	int planCounter;

};

fRgi::fRgi(){
	actions = nh.subscribe("testActions",100, &fRgi::testActionsCallback, this);
	endOfAction_ = nh.subscribe("endOfAction",100,&fRgi::endOfActionCallback,this);
	abortClient = nh.serviceClient<g2s_interface::abort_Action>("abort_Action",0);
	goto1Client = nh.serviceClient<g2s_interface::runGOTO_WAYPOINT>("runGOTO_WAYPOINT",0);
	//goto2Client = nh.serviceClient<g2s_interface::runGOTO_WAYPOINT>("runGOTO_WAYPOINT",0);
	stopClient 	= nh.serviceClient<g2s_interface::abort_Action>("stop_Action",0);
	endOfActionBool = true;
	planCounter = 1;
	isRunningPath = false;
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
	switch(action)
	{
		case ACTION_DEF_ABORT 	: abortCurrentMission(); break;
		case ACTION_DEF_STOP	: stopCurrentAction(); break;
		case ACTION_DEF_GOTO1	: gotoWay1(); break;
		case ACTION_DEF_GOTO2	: gotoWay2(); break;
		case ACTION_DEF_GOTO3	: gotoWay3(); break;
		case ACTION_DEF_GOTO4	: gotoWay4(); break;
		case ACTION_DEF_PLAN1	: executePlan1(); isRunningPath=true; break;
		default 				: ROS_INFO("Unknown action command");
	}

}

void fRgi::abortCurrentMission(){
	g2s_interface::abort_Action ab;
	if (abortClient.call(ab))
	{
		ROS_INFO("Abort Successful");
	}
	isRunningPath = false;
}

void fRgi::stopCurrentAction(){
	g2s_interface::abort_Action ab;
	if (stopClient.call(ab))
	{
		ROS_INFO("Stop Successful");
	}
	isRunningPath = false;
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
	endOfActionBool = false;

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
	desiredPos.x = 50;
	desiredPos.y = 0;
	desiredPos.z = 0;

	go.request.waypointPosition = desiredPos;
	endOfActionBool = false;

	if (goto1Client.call(go))
	{
		ROS_INFO("Go to waypoint 2, successfully sent.");
	}

}

void fRgi::gotoWay3(){
	g2s_interface::runGOTO_WAYPOINT go;
	go.request.modeId = 1;

	//specify a point
	geometry_msgs::Point desiredPos;
	desiredPos.x = 50;
	desiredPos.y = 50;
	desiredPos.z = 0;

	go.request.waypointPosition = desiredPos;
	endOfActionBool = false;

	if (goto1Client.call(go))
	{
		ROS_INFO("Go to waypoint 3, successfully sent.");
	}

}

void fRgi::gotoWay4(){
	g2s_interface::runGOTO_WAYPOINT go;
	go.request.modeId = 1;

	//specify a point
	geometry_msgs::Point desiredPos;
	desiredPos.x = 0;
	desiredPos.y = 50;
	desiredPos.z = 0;

	go.request.waypointPosition = desiredPos;
	endOfActionBool = false;


	if (goto1Client.call(go))
	{
		ROS_INFO("Go to waypoint 4, successfully sent.");
	}

}

void fRgi::executePlan1(){

	switch(planCounter){
		case 1 : gotoWay1(); break;
		case 2 : gotoWay2(); break;
		case 3 : gotoWay3(); break;
		case 4 : gotoWay4(); break;
		case 5 : gotoWay1(); break;
		default : planCounter = 1; isRunningPath=false; break;
	}
	
}

void fRgi::endOfActionCallback(const g2s_interface::endOfAction& msg){
	endOfActionBool = true;
	planCounter++;


	if (isRunningPath && msg.endCode==1)  
	{
		ros::Duration(1).sleep();
		executePlan1();
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
