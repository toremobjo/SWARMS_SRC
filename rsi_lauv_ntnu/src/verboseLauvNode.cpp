#include <ros/ros.h>
#include "g2s_interface/endOfAction.h"
#include "rsi_lauv_ntnu/testMsgRsiLauv.h"

static std::string nodeName = "verboseLauvNode";

void endOfAction(const g2s_interface::endOfAction& msg)
{
	ROS_INFO("[%s] endOfAction (actionId:%d, endCode:%d)",
		nodeName.c_str(),msg.actionId,msg.endCode);
}

void testMsgRsiLauv(const rsi_lauv_ntnu::testMsgRsiLauv& msg)
{
	// ROS_INFO("[%s] testMsgRsiLauv (txt1:%s, txt2:%s, val1: %f, val2:%f)",
	// 	nodeName.c_str(),msg.txt1.c_str(),msg.txt1.c_str(),msg.val1,msg.val2);
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, nodeName, ros::init_options::AnonymousName);
	ros::NodeHandle nh;

	ros::Subscriber sub1 = nh.subscribe("testMsgRsiLauv_0",1000,&testMsgRsiLauv);
	ros::Subscriber sub3 = nh.subscribe("endOfAction",1000,&endOfAction);

	//ROS_INFO("[%s] ... Coucou ROS ...",nodeName.c_str());
  ros::spin();
  return 0;
}
