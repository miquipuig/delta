// robot geometry
//(look at pics above for explanation)

#include "delta_node.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include <sstream>
#include "std_msgs/UInt16MultiArray.h"
#include <vector>
#include <cmath>
#include "std_msgs/Bool.h"

double xIncfix;
double yIncfix;
bool mode1=true;
bool mode2=false;
double alfa = 55;
#define PI 3.14159265

//Fixar posició inicial que correspongui als angles de delta_kinematics
cv::Mat outPosition=(cv::Mat_<double>(3,1)<< 0,0,300);
double xInc=0;
double yInc=0;
double zInc=0;
int totaltime=2.5; //segons
const int rate =60; //Hz
const double k=10; //Velocitat
int prod=rate*totaltime;
int count = 0;
 void chatterCallback(const geometry_msgs::Vector3& vector)
 {

   xIncfix= cos(alfa * PI / 180.0 )*vector.x - sin(alfa * PI / 180.0 )*vector.y;
   yIncfix=-sin(alfa * PI / 180.0 )*vector.x - cos(alfa * PI / 180.0 )*vector.y;

   //std::cout << "X:"<<xIncfix << std::endl;
   //std::cout << "Y:"<<yIncfix << std::endl;
 }

 void pushedCallback(const std_msgs::Bool& pushed)
 {
   if(pushed.data==1){

     if(mode1){
       std::cout << "Tornar a la base" << std::endl;
        xInc=(0- outPosition.at<double>(0,0))/prod;
        yInc=(0- outPosition.at<double>(1,0))/prod;
        zInc=(300- outPosition.at<double>(2,0))/prod;
       mode1=false;
       mode2=true;
       count=0;
     }else{
       std::cout << "Buscant monedes" << std::endl;
       mode1=true;
       mode2=false;
     }
   }
 }


int main(int argc, char **argv)
{

  ros::init(argc, argv, "delta_controlVector");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("direction", 10,chatterCallback);
  ros::Subscriber pus = n.subscribe("pushed", 10,pushedCallback);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Vector3>("position", 1);
  ros::Rate loop_rate(rate);


  while (ros::ok())
  {

        //std::cout << "X:"<<xIncfix << std::endl;
        //std::cout << "y:"<<yIncfix << std::endl;

      if(mode1){

        if(xIncfix>0.2){
          xIncfix=0.2;
        }

        if(yIncfix>0.2){
          yIncfix=0.2;
        }
        outPosition.at<double>(0,0) = outPosition.at<double>(0,0)+xIncfix*k;
        outPosition.at<double>(1,0) = outPosition.at<double>(1,0)+yIncfix*k;
        outPosition.at<double>(2,0) = outPosition.at<double>(2,0);

        xIncfix=0;
        yIncfix=0;
      }

      if(mode2){

        if(count<prod){


          outPosition.at<double>(0,0) = outPosition.at<double>(0,0)+xInc;
          outPosition.at<double>(1,0) = outPosition.at<double>(1,0)+yInc;
          outPosition.at<double>(2,0) = outPosition.at<double>(2,0)+zInc;

        count+=1;
      }

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
