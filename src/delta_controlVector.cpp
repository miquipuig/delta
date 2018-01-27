// robot geometry
//(look at pics above for explanation)

#include "delta_node.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include <sstream>
#include "std_msgs/UInt16MultiArray.h"
#include <vector>
#include <cmath>

double xIncfix;
double yIncfix;



cv::Mat inPosition=(cv::Mat_<double>(3,1)<< 0,0,0);

//Fixar posició inicial que correspongui als angles de delta_kinematics
cv::Mat outPosition=(cv::Mat_<double>(3,1)<< 0,0,0);

 void chatterCallback(const geometry_msgs::Vector3& vector)
 {
   inPosition=(cv::Mat_<double>(3,1)<< vector.x,vector.y,vector.z);
   yIncfix=vector.y;
   xIncfix=vector.x;
 }

int main(int argc, char **argv)
{

  const int rate =5; //Hz
  const double k=0.5;

  ros::init(argc, argv, "delta_controlVector");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("ray_director", 10,chatterCallback);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Vector3>("position", 1);
  ros::Rate loop_rate(rate);


  while (ros::ok())
  {

        outPosition.at<double>(0,0) = outPosition.at<double>(0,0)+xIncfix*k;
        outPosition.at<double>(1,0) = outPosition.at<double>(1,0)+yIncfix*k;
        outPosition.at<double>(2,0) = outPosition.at<double>(2,0);




  }

    geometry_msgs::Vector3 msg;
    msg.x = outPosition.at<double>(0, 0);
    msg.y = outPosition.at<double>(1, 0);
    msg.z = outPosition.at<double>(2, 0);
    chatter_pub.publish(msg);

    //ROS_INFO("%s", msg.data.c_str());

    ros::spinOnce();

    loop_rate.sleep();


}