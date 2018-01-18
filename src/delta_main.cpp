
//ros dependencies
#include "delta_node.h"

//node main
int main(int argc, char **argv)
{
      //init ros
      ros::init(argc, argv, "delta");

      //create ros wrapper object
      RosImgProcessorNode imgp;

      //set node loop rate
      ros::Rate loopRate(imgp.getRate());

      //node loop
      while ( ros::ok() )
      {
            //execute pending callbacks
            ros::spinOnce();

            //do things
            imgp.process();

            //publish things
            imgp.publish();

            //relax to fit output rate
            loopRate.sleep();
      }

      //exit program
      return 0;
}
