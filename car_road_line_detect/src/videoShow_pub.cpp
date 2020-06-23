#include "ros/ros.h"
#include "std_msgs/String.h"

#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include <sstream>


using namespace cv;
using namespace std;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[])
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "videoShow_pub");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;

	/**
	 * The advertise() function is how you tell ROS that you want to
	 * publish on a given topic name. This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing. After this advertise() call is made, the master
	 * node will notify anyone who is trying to subscribe to this topic name,
	 * and they will in turn negotiate a peer-to-peer connection with this
	 * node.  advertise() returns a Publisher object which allows you to
	 * publish messages on that topic through a call to publish().  Once
	 * all copies of the returned Publisher object are destroyed, the topic
	 * will be automatically unadvertised.
	 *
	 * The second parameter to advertise() is the size of the message queue
	 * used for publishing messages.  If messages are published more quickly
	 * than we can send them, the number here specifies how many messages to
	 * buffer up before throwing some away.
	 */
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	ros::Rate loop_rate(10);

	/**
	 * A count of how many messages we have sent. This is used to create
	 * a unique string for each message.
	 */
	// 定义节点句柄   
    image_transport::ImageTransport it(n);
    image_transport::Publisher image_pub = it.advertise("video_image", 1);
    sensor_msgs::ImagePtr msg;
        
    // opencv准备读取视频
    VideoCapture video;
    video.open("/home/pa/slam_study_on_linux/car_video3.mp4");  

    if( !video.isOpened() )
    {
        ROS_INFO("Read Video failed!\n");
        return 0;
    }
    Mat frame;
	int count = 0;
	while (ros::ok())
	{
		
		/**
		 * This is a message object. You stuff it with data, and then publish it.
		 */
		//std_msgs::String msg;

		
		video >> frame;
        if( frame.empty() )
            break;
		//resize(frame, frame, Size(1280,1280*frame.rows/frame.cols));
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        //cv::imshow("view", frame);
		//waitKey(1);
		std::stringstream ss;
		cout << "frame size:"<<frame.rows<<" "<<frame.cols << endl;
		//msg.data = ss.str();

		//ROS_INFO("%s","hhhh");

		/**
		 * The publish() function is how you send messages. The parameter
		 * is the message object. The type of this object must agree with the type
		 * given as a template parameter to the advertise<>() call, as was done
		 * in the constructor above.
		 */
		image_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}