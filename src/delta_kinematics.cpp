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

const float e = 110;     // Longitud del lado del triángulo equilátero pequeño (el que se moverá).
const float f = 230;     //MODIFICARRRRR Longitud del lado del triángulo equilátero grande  (el que estará estático).
const float re = 245;    // Longitud del brazo que va unido al triángulo pequeño y a 'rf'.
const float rf = 150;    // Longitud del pequeño brazo que está unido al eje del servo y a 're'.

const float sqrt3 = sqrt(3.0);
const float pi = 3.141592653;    // Pi
const float sin120 = sqrt3/2.0;
const float cos120 = -0.5;
const float tan60 = sqrt3;
const float sin30 = 0.5;
const float tan30 = 1/sqrt3;

float theta_1;
float theta_2;
float theta_3;


//inversa

// forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
 // returned status: 0=OK, -1=non-existing position
 int delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0) {
     float t = (f-e)*tan30/2;
     float dtr = pi/(float)180.0;

     theta1 *= dtr;
     theta2 *= dtr;
     theta3 *= dtr;

     float y1 = -(t + rf*cos(theta1));
     float z1 = -rf*sin(theta1);

     float y2 = (t + rf*cos(theta2))*sin30;
     float x2 = y2*tan60;
     float z2 = -rf*sin(theta2);

     float y3 = (t + rf*cos(theta3))*sin30;
     float x3 = -y3*tan60;
     float z3 = -rf*sin(theta3);

     float dnm = (y2-y1)*x3-(y3-y1)*x2;

     float w1 = y1*y1 + z1*z1;
     float w2 = x2*x2 + y2*y2 + z2*z2;
     float w3 = x3*x3 + y3*y3 + z3*z3;

     // _x = (a1*z + b1)/dnm
     float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
     float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;

     // y = (a2*z + b2)/dnm;
     float a2 = -(z2-z1)*x3+(z3-z1)*x2;
     float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;

     // a*z^2 + b*z + c = 0
     float a = a1*a1 + a2*a2 + dnm*dnm;
     float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
     float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);

     // discriminant
     float d = b*b - (float)4.0*a*c;
     if (d < 0) return -1; // non-existing point

     z0 = -(float)0.5*(b+sqrt(d))/a;
     x0 = (a1*z0 + b1)/dnm;
     y0 = (a2*z0 + b2)/dnm;
     return 0;
 }








int delta_calcAngleYZ(float x0, float y0, float z0, float &theta) {
    float y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
    y0 -= 0.5 * 0.57735    * e;    // shift center to edge
    // z = a + b*y
    float a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
    float b = (y1-y0)/z0;
    // discriminant
    float d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf);
    if (d < 0) return -1; // non-existing point
    float yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
    float zj = a + b*yj;
    theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0); // Si se cumple que yj>y1 entonces suma "180.0", de lo contrario no suma nada "0.0".
    return 0;
}

int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3) {
    theta1 = theta2 = theta3 = 0;
    int status = delta_calcAngleYZ(x0, y0, z0, theta1);
    if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg
    if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg
    return status;
}

//thetas inicials
std::vector<int> thetas = { 130, 130 , 130};


 void chatterCallback(const geometry_msgs::Vector3& vector)
 {

float th1;
float th2;
float th3;


if(vector.x!=0 || vector.y!=0 || vector.z!=0){

         if(!delta_calcInverse(vector.x,vector.y,-vector.z,th1,th2,th3)==0){
           //ROS_INFO("No hi ha inversa possible");
         }
         else{
        theta_1=180-th1;
        theta_2=180-th2;
        theta_3=180-th3;
         //ROS_INFO("theta1: [%f]", theta1);
         //ROS_INFO("theta2: [%f]", theta2);
         //ROS_INFO("theta3: [%f]", theta3);
       }
  }
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "delta_kinematics");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("position", 10,chatterCallback);
  ros::Publisher chatter_pub = n.advertise<std_msgs::UInt16MultiArray >("servo", 10);

  ros::Rate loop_rate(60);



float xi =4;
float yi =4;
float zi =4;

  int count = 0;
  while (ros::ok())
  {



    //std::cout << "Solucio?"<< delta_calcForward(50,50,50,xi,yi,zi) << std::endl;
    //std::cout << "x"<< xi << std::endl;
    //std::cout << "y"<< yi << std::endl;
    //std::cout << "z"<< zi << std::endl;


    std_msgs::UInt16MultiArray msg;
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = 3;
    //msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "thetas";
    msg.data.clear();
    if(theta_1<0){
      thetas[0]=0;

    }else{
      thetas[0]=theta_1;
    }

    if(theta_2<0){
      thetas[1]=0;

    }else{
      thetas[1]=theta_2;
    }

    if(theta_3<0){
      thetas[2]=0;

    }else{
      thetas[2]=theta_3;
    }
     //std::cout << "theta1:"<<theta1 << std::endl;
      //std::cout << "theta2:"<<theta2 << std::endl;
      // std::cout << "theta3:"<<theta3 << std::endl;
    //thetas[1]=theta2;
    //thetas[2]=theta3;
    msg.data.insert(msg.data.end(), thetas.begin(), thetas.end());

    chatter_pub.publish(msg);

    //ROS_INFO("%s", msg.data.c_str());

    ros::spinOnce();

    loop_rate.sleep();

  }
}
