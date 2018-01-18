#include "delta_node.h"


const int GAUSSIAN_BLUR_SIZE = 7;
const double GAUSSIAN_BLUR_SIGMA = 2;
const double CANNY_EDGE_TH = 150;
const double HOUGH_ACCUM_RESOLUTION = 2;
const double MIN_CIRCLE_DIST = 40;
const double HOUGH_ACCUM_TH = 70;
const int MIN_RADIUS = 20;
const int MAX_RADIUS = 100;


RosImgProcessorNode::RosImgProcessorNode() :
    nh_(ros::this_node::getName()),
    img_tp_(nh_)
{
	//loop rate [hz], Could be set from a yaml file
	rate_=10;

	//sets publishers
	image_pub_ = img_tp_.advertise("image_out", 100);

	//sets subscribers
	image_subs_ = img_tp_.subscribe("image_in", 1, &RosImgProcessorNode::imageCallback, this);
	camera_info_subs_ = nh_.subscribe("camera_info_in", 100, &RosImgProcessorNode::cameraInfoCallback, this);
}

RosImgProcessorNode::~RosImgProcessorNode()
{
    //
}

void RosImgProcessorNode::process()
{
    cv::Rect_<int> box;
    cv::Point center;
    cv::Mat gray_image;
    int radius;
    std::vector<cv::Vec3f> circles;
    cv::Mat K = (cv::Mat_<double>(3,3) << 538.8181225518491, 0, 538.8181225518491, 0, 538.8181225518491, 538.8181225518491, 0, 0, 1);
    cv::Mat u=(cv::Mat_<double>(3,1)<< 0,0,0);
    cv::Mat d;
    double xcenter=640/2;
    double ycenter=480/2;
    cv::Point centerp = cv::Point(xcenter, ycenter);
    //check if new image is there
    if ( cv_img_ptr_in_ != nullptr )
    {


        //copy the input image to the out one
        cv_img_out_.image = cv_img_ptr_in_->image;

		//find the ball
		//TODO
    circles.clear();
    cv::cvtColor(cv_img_ptr_in_->image, gray_image, CV_BGR2GRAY);
    cv::GaussianBlur( gray_image, gray_image, cv::Size(GAUSSIAN_BLUR_SIZE, GAUSSIAN_BLUR_SIZE), GAUSSIAN_BLUR_SIGMA );
    cv::HoughCircles( gray_image, circles, CV_HOUGH_GRADIENT, HOUGH_ACCUM_RESOLUTION, MIN_CIRCLE_DIST, CANNY_EDGE_TH, HOUGH_ACCUM_TH, MIN_RADIUS, MAX_RADIUS );

		//find the direction vector
		//TODO
     cv::Mat Kinv= K.inv();
     //exemple de u
     //cv::Mat u=(cv::Mat_<double>(3,1)<< 1,1,0);
     //cv::Mat d= Kinv*u;






    for(unsigned int ii = 0; ii < circles.size(); ii++ )
    {

        if ( circles[ii][0] != -1 )
        {
                //std::cout << "Circulo: " << circles[ii][0] <<";"<< circles[ii][1]<<";"<< circles[ii][2]<<std::endl;


                u=(cv::Mat_<double>(3,1)<< circles[ii][0] -xcenter, circles[ii][0] - ycenter,0);

                //imprimeix coordenades Ray director
                d= Kinv*u;
                std::cout <<"Ray Director"<< d << std::endl;

                center = cv::Point(cvRound(circles[ii][0]), cvRound(circles[ii][1]));
                radius = cvRound(circles[ii][2]);
                cv::circle(cv_img_out_.image, center, 5, cv::Scalar(0,0,255), -1, 8, 0 );// circle center in green
                cv::circle(cv_img_out_.image, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );// circle perimeter in red
                //vector Ray director
                cv::line(cv_img_out_.image,centerp,center,cv::Scalar(0,0,255), 8); //linea


        }
    }

    }

    //reset input image
    cv_img_ptr_in_ = nullptr;
}

void RosImgProcessorNode::publish()
{
    //image_raw topic
	if(cv_img_out_.image.data)
	{
	    cv_img_out_.header.seq ++;
	    cv_img_out_.header.stamp = ros::Time::now();
	    cv_img_out_.header.frame_id = "camera";
	    cv_img_out_.encoding = img_encoding_;
	    image_pub_.publish(cv_img_out_.toImageMsg());
	}
}

double RosImgProcessorNode::getRate() const
{
    return rate_;
}

void RosImgProcessorNode::imageCallback(const sensor_msgs::ImageConstPtr& _msg)
{
    try
    {
        img_encoding_ = _msg->encoding;//get image encodings
        cv_img_ptr_in_ = cv_bridge::toCvCopy(_msg, _msg->encoding);//get image
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("RosImgProcessorNode::image_callback(): cv_bridge exception: %s", e.what());
        return;
    }
}

void RosImgProcessorNode::cameraInfoCallback(const sensor_msgs::CameraInfo & _msg)
{
	matrixP_ = (cv::Mat_<double>(3,3) << _msg.P[0],_msg.P[1],_msg.P[2],
                                        _msg.P[3],_msg.P[4],_msg.P[5],
                                        _msg.P[6],_msg.P[7],_msg.P[8]);
	//std::cout << matrixP_ << std::endl;
}
