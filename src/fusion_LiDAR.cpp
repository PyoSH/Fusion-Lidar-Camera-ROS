#include <ros/ros.h>
#include <mutex>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// ///////////////////////////////////////////////////////////////////////
// topic: /test_lidar
// frame: sensor_frame
// multi-topic asynchronized
// ///////////////////////////////////////////////////////////////////////
using namespace std;


static bool IS_IMAGE_CORRECTION = true;

std::mutex mut_img;
std::mutex mut_pc;


//포인트클라우드 메시지 포함 xyz和intensity
pcl::PointCloud<pcl::PointXYZI>::Ptr raw_pcl_ptr(new pcl::PointCloud<pcl::PointXYZI>); 
pcl::PointCloud<pcl::PointXYZI>::Ptr copy_raw_pcl_ptr(new pcl::PointCloud<pcl::PointXYZI>); 

cv::Mat image_color;
cv::Mat copy_image_color;

// 이미지, 라이다 들어오는지 확인
bool is_rec_image = false;
bool is_rec_LiDAR = false;

typedef pcl::PointXYZRGB PointType;


class ImageLiDARFusion
{
public:
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  ros::Subscriber LiDAR_sub;
  image_transport::Publisher image_pub;
  ros::Publisher LiDAR_pub;
  cv_bridge::CvImagePtr cv_ptr;
  sensor_msgs::PointCloud out_pointcloud;
  sensor_msgs::PointCloud2 colored_msg; 
      
  pthread_t  tids1_;

public:
  cv::Mat transform_matrix;    /**< from globol to image coordinate */
	cv::Mat intrinsic_matrix;    /**< from local to image coordinate  */
	cv::Mat extrinsic_matrix;    /**< from global to local coordinate */
	cv::Mat dist_matrix;         /**< dist parameters  */
  int img_height;
  int img_width;

public:
  ImageLiDARFusion():it(nh)
  {
    ROS_INFO("------------ intialize ----------\n");
    this->set_param();

    // image_sub = it.subscribe("/camera/stopline/image_raw", 100, &ImageLiDARFusion::imageCallback, this);
    image_sub = it.subscribe("/camera_fusion_test", 100, &ImageLiDARFusion::imageCallback, this);
    // LiDAR_sub = nh.subscribe("/livox/lidar", 100, &ImageLiDARFusion::LiDARCallback, this);
    LiDAR_sub = nh.subscribe("/velodyne_points", 100, &ImageLiDARFusion::LiDARCallback, this);
   
    image_pub = it.advertise("/test_camera", 10);    
    LiDAR_pub = nh.advertise<sensor_msgs::PointCloud2>("test_LiDAR", 10);
    int ret1 = pthread_create(&tids1_, NULL,  publish_thread, this); 

    ROS_INFO("START LISTENING\n");
  };

  ~ImageLiDARFusion()
  {
  };

  void set_param();
  void LiDARCallback(const sensor_msgs::PointCloud2ConstPtr & msg);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  static void * publish_thread(void *  this_sub);
};

void * ImageLiDARFusion::publish_thread(void * args)
{
  ImageLiDARFusion * this_sub = (ImageLiDARFusion *) args; //**point3**：입력된 void*매개변수를 클래스 객체(ImageLiDARFusion*)로 반환 
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
  
    if (is_rec_image && is_rec_LiDAR)
    {
      mut_pc.lock();
      pcl::copyPointCloud (*raw_pcl_ptr, *copy_raw_pcl_ptr);
      mut_pc.unlock();

      mut_img.lock();
      copy_image_color = image_color.clone();
      mut_img.unlock();

      pcl::PointCloud<PointType>::Ptr pc_xyzrgb(new pcl::PointCloud<PointType>);
      const int size = copy_raw_pcl_ptr->points.size(); 
      
      cout<<"point cloud size: "<<size<< endl;;

      for (int i = 0; i < size; i++)
      {
        // project get the photo coordinate
        // pcl::PointXYZRGB pointRGB;
        PointType pointRGB;

        pointRGB.x = copy_raw_pcl_ptr->points[i].x;
        pointRGB.y = copy_raw_pcl_ptr->points[i].y;
        pointRGB.z = copy_raw_pcl_ptr->points[i].z;
        double a_[4] = { pointRGB.x, pointRGB.y, pointRGB.z, 1.0 };
        cv::Mat pos(4, 1, CV_64F, a_);
        cv::Mat newpos(this_sub->transform_matrix * pos);
        float x = (float)(newpos.at<double>(0, 0) / newpos.at<double>(2, 0));
        float y = (float)(newpos.at<double>(1, 0) / newpos.at<double>(2, 0));
        // cout << "x: "<<x <<"  / y: "<<y<<endl;

        // Trims viewport according to image size
        if (pointRGB.x >= 0)
        {
          if (x >= 0 && x < this_sub->img_width && y >= 0 && y < this_sub->img_height)
          {
            //  imread BGR（BITMAP）
            int row = int(y);
            int column = int(x);
            cout << "row: "<<row <<"  / column: "<<column<<endl;
            pointRGB.r = copy_image_color.at<cv::Vec3b>(row, column)[2];
            pointRGB.g = copy_image_color.at<cv::Vec3b>(row, column)[1];
            pointRGB.b = copy_image_color.at<cv::Vec3b>(row, column)[0];
           
            // pointRGB.intensity = copy_raw_pcl_ptr->points[i].intensity; //이전 포인트 클라우드 상속 : intensity
            pc_xyzrgb->push_back(pointRGB);
          }
        }
      }
      pc_xyzrgb->width = 1;
      pc_xyzrgb->height = pc_xyzrgb->points.size();
      pcl::toROSMsg(*pc_xyzrgb,  this_sub->colored_msg);  // 将点云转化为ROS消息发布
      this_sub->colored_msg.header.frame_id = "sensor_frame";
      this_sub->colored_msg.header.stamp = ros::Time::now(); 
      this_sub->LiDAR_pub.publish(this_sub->colored_msg); 
      loop_rate.sleep();
    }
  }
}


void ImageLiDARFusion::set_param() 
{
  // extrinsic matrix parameters
  XmlRpc::XmlRpcValue param_list;
  std::vector<double> Extrin_matrix;
  if(!nh.getParam("/fusion_LiDAR/CameraExtrinsicMat/data", param_list))
      ROS_ERROR("Failed to get extrinsic parameter.");
  ROS_INFO("\n get extrinsic parameter:");
  for (size_t i = 0; i < param_list.size(); ++i) 
  {
      XmlRpc::XmlRpcValue tmp_value = param_list[i];
      if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
      {    
        Extrin_matrix.push_back(double(tmp_value));
        ROS_INFO("PARAME SIZE = %f", double(tmp_value));
      }
  }
  // Intrinsic matrix parameters
  std::vector<double> Intrinsic;
  if(!nh.getParam("/fusion_LiDAR/CameraMat/data", param_list))
      ROS_ERROR("Failed to get intrinsic parameter.");
  ROS_INFO("\n get intrinsic parameter:");
  for (size_t i = 0; i < param_list.size(); ++i) 
  {
      XmlRpc::XmlRpcValue tmp_value = param_list[i];
      if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
      {    
        Intrinsic.push_back(double(tmp_value));
        ROS_INFO("PARAME SIZE = %f", double(tmp_value));
      }
  }


  // 5 distortion parameters
  std::vector<double> dist;
  if(!nh.getParam("/fusion_LiDAR/DistCoeff/data", param_list))
      ROS_ERROR("Failed to get distortion parameter.");
  ROS_INFO("\n get distortion parameter:");
  for (size_t i = 0; i < param_list.size(); ++i) 
  {
      XmlRpc::XmlRpcValue tmp_value = param_list[i];
      if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
      {    
        dist.push_back(double(tmp_value));
        ROS_INFO("PARAME SIZE = %f", double(tmp_value));
      }
  }
  
  // img size
  std::vector<int> img_size;
  if(!nh.getParam("/fusion_LiDAR/ImageSize", param_list))
      ROS_ERROR("Failed to get extrinsic parameter.");
  ROS_INFO("\n get image size:");
  for (size_t i = 0; i < param_list.size(); ++i) 
  {
      XmlRpc::XmlRpcValue tmp_value = param_list[i];
      if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeInt)
      {    
        img_size.push_back(int(tmp_value));
        ROS_INFO("PARAME SIZE = %d", int(tmp_value));
      }
  }
  img_width = img_size[0];
  img_height = img_size[1];
  // convert cv::Mat
  cv::Mat dist_array(5, 1, CV_64F, &dist[0]);
  this->dist_matrix = dist_array.clone();

  
  cv::Mat Int(3, 3, CV_64F, &Intrinsic[0]);
  this->intrinsic_matrix = Int.clone();

  
  cv::Mat ext_(4, 4, CV_64F, &Extrin_matrix[0]);
  cv::Mat invRt = ext_(cv::Rect(0, 0, 3, 3));
  cv::Mat R = invRt.t();
  cv::Mat invT = -R * ext_(cv::Rect(3, 0, 1, 3));
  cv::hconcat(R, invT, this->extrinsic_matrix);
  // transform matrix: from global coordinate to image coordinate
  this->transform_matrix = Int * this->extrinsic_matrix;
}



void ImageLiDARFusion::LiDARCallback(const sensor_msgs::PointCloud2ConstPtr & msg)
{
  mut_pc.lock();
  pcl::fromROSMsg(*msg, *raw_pcl_ptr);	
  mut_pc.unlock();
  is_rec_LiDAR = true;
}


void ImageLiDARFusion::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // image correction
  if(IS_IMAGE_CORRECTION)
  {
    mut_img.lock();
    cv::undistort(cv_ptr->image, image_color, this->intrinsic_matrix, this->dist_matrix);
    // image_color = cv_ptr->image.clone(); 
    cv_ptr->image = image_color.clone();
    mut_img.unlock();

  }
  else
  {
    mut_img.lock();  
    image_color = cv_ptr->image.clone();
    mut_img.unlock();
  }
  is_rec_image = true;
  sensor_msgs::ImagePtr linshi_msg = cv_ptr->toImageMsg();
  linshi_msg->header.stamp = ros::Time::now();
  image_pub.publish(linshi_msg);
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "fusion");
  if (argc != 2){ROS_ERROR("need is_correct as argument"); return -1;}; 
  
  if (strcmp(argv[1], "true") == 0){IS_IMAGE_CORRECTION = true; ROS_INFO("correct image");}
  else {IS_IMAGE_CORRECTION = false;ROS_INFO("don't correct image");}
  ImageLiDARFusion ic;
  ros::MultiThreadedSpinner spinner(4);  //노드 멀티쓰레딩
  spinner.spin();   
  // ros::spin();
  return 0;
}