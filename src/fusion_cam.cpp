// K-BUB team, PyoSH retouched
// preprocessing 
// 0409 update : 1. RGB_mean -> IMG_mean


#include "stopline_func.h"
#include "stopline_func.cpp"

using namespace cv;
using namespace std;

WeAreVision WAV;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "stopline_publisher");
	// ros::NodeHandle nh;
	ros::NodeHandle nh1;
	ros::Publisher stopline_pub = nh1.advertise<std_msgs::Float64>("stopline", 100); //int형 메시지
	image_transport::ImageTransport it(nh1);
	image_transport::Publisher image_raw_pub = it.advertise("/camera_fusion_test", 100); //카메라에서 이미지 읽어서 송신
	sensor_msgs::ImagePtr msg1;
	ros::Rate loop_rate(50);

	VideoCapture cap1(0);
	cap1.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	cap1.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	while (ros::ok())
	{
		
		cap1 >> WAV.frame1;
		msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", WAV.frame1).toImageMsg();
		image_raw_pub.publish(msg1);
		if (!cap1.isOpened())
		{
			cerr << "finish!\n"
				 << endl;
		}

		imshow("final", WAV.frame1);

		std_msgs::Float64 msg;
		
		stopline_pub.publish(msg);

		waitKey(1);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}