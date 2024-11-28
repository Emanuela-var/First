#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
 
class ImageProcessorNode : public rclcpp::Node {
public:
  ImageProcessorNode() : Node("opencv_image_processor") {
    // Subscriber per l'immagine simulata
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/videocamera", 10,
        std::bind(&ImageProcessorNode::image_callback, this, std::placeholders::_1));
 
    // Publisher per l'immagine processata
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/processed_image", 10);
  }
 
private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

	
    // Setup SimpleBlobDetector parameters.
    cv::SimpleBlobDetector::Params params;
    

    // Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 255;
    
    //Filter by color
    params.filterByColor=false;
    params.blobColor=0;

    // Filter by Area.
    params.filterByArea = false;
    params.minArea = 0.1;
    //params.maxArea=5000;
    
    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.8;
    
    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.9;
    
    // Filter by Inertia
    params.filterByInertia = false;
    params.minInertiaRatio = 0.01;
    
    //Mat im=imread(cv_ptr->image,IMREAD_GRAYSCALE);

    // Set up detector with params
    //SimpleBlobDetector detector(params);
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    
    // You can use the detector this way
    // detector.detect( im, keypoints);

    std::vector<cv::KeyPoint> keypoints;
    detector->detect(cv_ptr->image,keypoints);
    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    cv::Mat im_with_keypoints;
    cv::drawKeypoints(cv_ptr->image, keypoints, im_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);



    // // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(0,255,0));

    // Update GUI Window
    cv::imshow("Image window", im_with_keypoints);
    cv::waitKey(3);

    auto processed_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", im_with_keypoints).toImageMsg();
    publisher_->publish(*processed_msg);
  }



  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

};

 
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageProcessorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}