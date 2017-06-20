#include <ros/ros.h>
#include <rsi_lauv_ntnu/testSrvRsiLauv.h>

#include <utilFctn.hpp>

static std::string nodeName = "zeroLauvNode";

int main (int argc, char **argv)
{
	ros::init(argc, argv, nodeName, ros::init_options::AnonymousName);
	ros::NodeHandle nh;
	ROS_INFO("[%s] ... Coucou ROS ...",nodeName.c_str());

	ros::ServiceClient cli = nh.serviceClient<rsi_lauv_ntnu::testSrvRsiLauv>("testSrvRsiLauv_0");
	rsi_lauv_ntnu::testSrvRsiLauv::Request req;
	rsi_lauv_ntnu::testSrvRsiLauv::Response res;

	ros::Duration timeout(60);
	if (cli.waitForExistence())
	{
		int cpt(0);
		utilfctn::TicToc chrono1;
		ros::Rate rate(0.1); // Hz
		while(ros::ok() && cpt<50)
		{
			// cpt = cpt+1;
			// req.ind1 = cpt;
			// req.ind1 = 33;
			//
			// chrono1.start();
			// ROS_INFO("[%s] testSrvRsiLauv_0 request sent",nodeName.c_str());
			// bool success = cli.call(req, res);
			// ROS_INFO("[%s] testSrvRsiLauv_0 response received",nodeName.c_str());
			// chrono1.stop();
			// if (success)
			// 	ROS_INFO("[%s] testSrvRsiLauv_0 OK: %d, %d (%.3f sec.)",nodeName.c_str(),int(res.val1),int(res.val2),chrono1.getDtSec());
			// else
			// 	ROS_ERROR("[%s] testSrvRsiLauv_0 FAULT!",nodeName.c_str());

			rate.sleep();
		}
	}
	return 0;
}
