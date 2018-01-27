// robot geometry
//(look at pics above for explanation)

#include "delta_node.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include <sstream>
#include "std_msgs/UInt16MultiArray.h"
#include <vector>
#include <cmath>




cv::Mat inPosition=(cv::Mat_<double>(3,1)<< 0,0,315);

//Fixar posició inicial que correspongui als angles de delta_kinematics
cv::Mat outPosition=(cv::Mat_<double>(3,1)<< 0,0,315);

 void chatterCallback(const geometry_msgs::Vector3& vector)
 {
   inPosition=(cv::Mat_<double>(3,1)<< vector.x,vector.y,vector.z);

 }

int main(int argc, char **argv)
{

  int rate =5; //Hz
  int totaltime=4; //segons
  int prod=rate*totaltime;

  ros::init(argc, argv, "delta_control");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("end_position", 10,chatterCallback);

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Vector3>("position", 1);


  ros::Rate loop_rate(rate);

  int count = 0;


  double xIncfix=(inPosition.at<double>(0,0)- outPosition.at<double>(0,0))/prod;
  double yIncfix=(inPosition.at<double>(1,0)- outPosition.at<double>(1,0))/prod;
  double zIncfix=(inPosition.at<double>(2,0)- outPosition.at<double>(2,0))/prod;

  while (ros::ok())
  {

    if(count<prod){

      double xInc=inPosition.at<double>(0,0)- outPosition.at<double>(0,0);
      double yInc=inPosition.at<double>(1,0)- outPosition.at<double>(1,0);
      double zInc=inPosition.at<double>(2,0)- outPosition.at<double>(2,0);

      //if(xInc>0.5 || xInc<0.5){
        outPosition.at<double>(0,0) = outPosition.at<double>(0,0)+xIncfix;
      //}
      //if(yInc>0.5 || yInc<0.5){
        outPosition.at<double>(1,0) = outPosition.at<double>(1,0)+yIncfix;
      //}
      //if(zInc>0.5 || zInc<0.5){
        outPosition.at<double>(2,0) = outPosition.at<double>(2,0)+zIncfix;
      //}

      count+=1;
      //ROS_INFO("midxIncfix: [%lf]",xIncfix );
  }else{
    //ROS_INFO("####Paso por aqui: [%lf]",xIncfix);
        count=0;
    //ROS_INFO("finalxIncfix: [%lf]",xIncfix );

    //Refixar objectiu
     xIncfix=(inPosition.at<double>(0,0)- outPosition.at<double>(0,0))/prod;
     yIncfix=(inPosition.at<double>(1,0)- outPosition.at<double>(1,0))/prod;
     zIncfix=(inPosition.at<double>(2,0)- outPosition.at<double>(2,0))/prod;
    //ROS_INFO("inicialxIncfix: [%lf]",xIncfix );
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
}
