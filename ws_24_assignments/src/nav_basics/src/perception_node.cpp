/*
    What this node does is the following: 
        -wait for the "start" message to be received in the /start_perception custom topic.
        -once the start is received, it start the perception by receiving data from the 
         \scan topic.
        -it start to perform detection of the cylinders in the following way:
            -we convert the distance scan data received in a cv::Mat object 
            -we remove the straight line in the image
            -we perform Hough transform in the remaining new cv::Mat object
                (This allows us to find the circles in the 2d data detcetions)
            -we repeat the same for each detection.
            -if the new scans received in the following 10 detections does not contain no new
             cylinder we assume the current number is certain. 
        -we publish the position of cylinders centers in the \cylinders topic.
*/

// Robotic Framework library.
#include "rclcpp/rclcpp.hpp"

// Messages libraries.
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"

// Transformation libraries.
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

// Computer vision libraries.
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp> 
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

// Language libraries.
#include <cmath>
#include <memory>
#include <atomic>
#include <vector>

// This namespaces are used to shorten the messages types.
using LaserScan = sensor_msgs::msg::LaserScan;
using PoseArray = geometry_msgs::msg::PoseArray;
using Pose = geometry_msgs::msg::Pose;

using std::placeholders::_1;
using std::placeholders::_2;


/*
  This function's objective is to remove the straight lines present in the lidar scan 
  detections.
  Args: 
    -the cv::Mat object representing the lidar scan data positions with white pixels.
  Returns: 
    -a cv Mat object without all the straight lines.
*/
cv::Mat remove_walls(const cv::Mat img) {
    // Create the output image.
    cv::Mat output = img.clone();

    // Detect straight lines.
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(
        img,
        lines,
        1,         // Rho .
        CV_PI/180, // Theta.
        15,        // Threshold.
        0,
        0
    );
 
    // Removing all the lines present in the image using two points and printing the
    // correspoding line in black.
    for(const cv::Vec2f &line : lines) {
        // Parameters of the line detected by HoughLines.
        float rho = line[0];
        float theta = line[1];

        // Define two points on the line.
        cv::Point pt1, pt2;
        double a = std::cos(theta), b = std::sin(theta);
        double x0 = a * rho, y0 = b * rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));

        // Remove line (print a black line).
        cv::line(output, pt1, pt2, cv::Scalar(0), cv::LINE_AA);
        
    }

    return output;
}



/*
  This function convert the laserscan message into an image using distances and 
   angles of collected data. 
  Args: 
    -The Laserscan sensor message.
    -The size of the square image to output (int).
    -The resolution (double).
  Retunrns: 
    -The converted cv::Mat object.
*/
cv::Mat convertScanToMat(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg, 
                         int img_size = 500, 
                         double resolution = 50.0)
{
    // Create a black image.
    cv::Mat scan_image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

    // Calculate the center of the image.
    const int origin_x = img_size / 2;
    const int origin_y = img_size / 2;

    double current_angle = msg->angle_min;

    // Loop through all the range readings.
    for (const float range : msg->ranges)
    {
        // Check for valid range data.
        if (range > msg->range_min && range < msg->range_max)
        {
            double x_meters = range * std::cos(current_angle);
            double y_meters = range * std::sin(current_angle);

            int pixel_col = origin_x + static_cast<int>(x_meters * resolution);
            int pixel_row = origin_y - static_cast<int>(y_meters * resolution); 

            // Check if the point is within the image bounds.
            if (pixel_col >= 0 && pixel_col < img_size &&
                pixel_row >= 0 && pixel_row < img_size)
            {
                // Set the pixel to white.
                scan_image.at<uchar>(pixel_row, pixel_col) = 255;
            }
        }
        
        // Move to the next angle.
        current_angle += msg->angle_increment;
    }

    return scan_image;
}



/* 
  This function return true if the two point aren't in each other proximity.
  Args: 
    -Point 1.
    -Point 2.
    -Minimum distance accepted between the points.
  Returns:
    -bool telling whether the two point are in each other proximity.
*/
bool check_proximity(cv::Point point_1, cv::Point point_2, int min_distance)
{
    // Computing the distance between the points.
    double dx = point_1.x - point_2.x;
    double dy = point_1.y - point_2.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    // Checking validity of the distance.
    if (distance < min_distance) {
        return true;
    }
    
    return false;
}



/*
  This rclcpp:Node is a perception node, its task is to perform detection of the cylinders 
  once the robot is moved in the desired position and orientation. 
  
  NOTE: we assumed that the shape that should be perceived by the 2 dimension scan of the 
        lidar, corresponding to the cylinders should be a circle.
        
  The steps performed by this node are the following: 
    -Activate a subscribtion to the /start_perception custom topic, which send a start message
     used to start performing detection in lidar messages.
    -Once this node is activated by the activation message, we start performing detection in the 
     lidar data message
    -Also for each detection, we convert the data of the center of the detected circles from 
     current frame to "odom" frame.

  --> So we needed two subscription and one publisher. 
  
  Also we used the data from tf topic to perform transformation of the center coordinates 

*/
class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode()
  : Node("perception_node"), detected_cylinders_count_(0)
  {

    // Subscription for the start perception signal, using the start_callback private function.
    start_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/start_perception", 
      10,
      std::bind(&PerceptionNode::start_callback, this, _1));

    // Subscription for the laserscan data, using the lidar_callback private function.
    sub_ = this->create_subscription<LaserScan>(
      "/scan", 
      10,
      std::bind(&PerceptionNode::lidar_callback, this, _1));

    // Publisher for the cylinders center coordinates.
    pub_ = this->create_publisher<PoseArray>("/cylinders", 10);

    // Defining the buffer, where we store the data read from tf topic.
    tf_buffer_ =
      std::make_unique < tf2_ros::Buffer > (this -> get_clock());

    // Casting the buffer to a tf listener.
    tf_listener_ =
      std::make_shared < tf2_ros::TransformListener > ( * tf_buffer_);

    // After initializing all the internal variable used for the tasks we print a
    // message to say the node is ready.
    RCLCPP_INFO(this->get_logger(), "Perception node ready. listening /scan .");
  }

  /*
    This function is the callback used to get the messages from the start_perception 
    topic, we expect to get a "start" std_msgs string data.
    Args: 
      -msg, the std_msgs string with the message sent. 
  */
  void start_callback(const std::shared_ptr<std_msgs::msg::String> msg)
  {
    // Checking to see if the message is "start" (the expected start message).
    if (msg->data == "start") {
      // Setting the active variable to true, to allow detection to start. 
      perception_active_ = true;
      RCLCPP_INFO(this->get_logger(), "Perception activated. Processing LiDAR scans.");
    } else {
      // Setting the active variable to false, to avoid detection to start.
      perception_active_ = false;
      RCLCPP_INFO(this->get_logger(), "Perception deactivated. Ignoring LiDAR scans.");
    }
  }

  /*
    This function is the lidar callback, which allow the perception node to perceive the 
    enviroment through the laserscan messages. Also this node performs detection of the 
    circles from the converted data.
    Args: 
      -msg, the laserscan message.
  */
  void lidar_callback(const std::shared_ptr<LaserScan> msg)
  {
    // Checking the active variable, to see if the robot is in the right position.
    if(!perception_active_){
        return;
    }
    // Setting variables for the conversion of data.
    int img_size = 500;
    double resolution = 50.0;
    // Converting the data from the topic into an image.
    cv::Mat scan_image = convertScanToMat(msg, 500, 50.0);
    cv::Mat dst = scan_image.clone();
    // Removing the walls from the image, all the straight lines.
    cv::Mat filtered_image = remove_walls(scan_image);

    // --- Circle detection ---.

    cv::Mat blurred_image;
    // Apply Gaussian blur to the filtered image to "thicken" remaining dots 
    // (cylinders) into blobs.
    cv::GaussianBlur(filtered_image, blurred_image, cv::Size(7, 7), 0, 0);

    // Output vector (x, y, radius).
    std::vector<cv::Vec3f> circles; 

    // Performing the actual circle detection using Hough transform.
    // The working paramters were determined empirically.
    cv::HoughCircles(
        blurred_image,        // Input image.
        circles,              // Output variable. 
        cv::HOUGH_GRADIENT,   // Method used for detections.  
        1,                       
        scan_image.rows / 16, 
        100, // Canny threshold.                    
        5,   // Hought threshold.                    
        1,   // Min radius of circles.                    
        10   // Max radius of circles.                  
    );

    // --- Visualize the results ---

    // Create a BGR image to draw colored circles on.
    cv::Mat circles_image;
    cv::cvtColor(scan_image, circles_image, cv::COLOR_GRAY2BGR);

    // Loop over all detected circles and draw them.
    std::vector<cv::Point> centers;
    for( size_t i = 0; i < circles.size(); i++ )
    {
        // Get center (x, y) and radius.
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        centers.push_back(center);
        int radius = cvRound(circles[i][2]);

        // Draw the circle center (green).
        cv::circle(circles_image, center, 3, cv::Scalar(0, 255, 0), 1);
        
        // Draw the circle outline (red).
        cv::circle(circles_image, center, radius, cv::Scalar(0, 0, 255), 1);
    }

    // Check if any circles were found.
    if (!circles.empty())
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Found: "<<circles.size()<<" cylinders");
    }else{
      RCLCPP_INFO(this->get_logger(), "Not found cylinders!");
    }
    // Add the circles to detection variable, if they were not present already.
    add_detections(centers);

    RCLCPP_INFO_STREAM(this->get_logger(), "Total of: "<<cylinders_positions_.size()<<" cylinders found!");
    //cv::imshow("Laser Scan Image", scan_image);
    // Printing in the screen a window with the detections.
    cv::imshow("Detected cylinders in green and red.", circles_image);

    // Iterating the blurred image, and make the blobs more intense, selecting the 
    // main points.
    for(int i = 0; i < blurred_image.rows; i++){
      for(int j = 0; j < blurred_image.cols; j++){
        // **Use uchar (unsigned char) for CV_8UC1 images.**

        // Here we use an empirical threshold to delete some weak points.
        if(blurred_image.at<uchar>(i,j) > 20){
          // Increasing intensivness of remaining points.
          blurred_image.at<uchar>(i,j) = (blurred_image.at<uchar>(i,j) + 80) % 256;
        }else{
          // Deleting below threshold points.
          blurred_image.at<uchar>(i,j) = 0;
        }
      }
    }

    // Printing on screen the blurred image after filtering out weak points and 
    // highlighting important points.
    cv::imshow("gaussian blur", blurred_image);
    cv::waitKey(1);

    // Calling a function to publish poses of detected centers in current detection
    // in the custom topic.
    publish_poses(msg, img_size, resolution, circles);

  }



  /*
    This function is used to pulish the poses of the detected centers in current cylinder
    detection. Also it convert the poses from current frame to "odom" frame.
    Args: 
      -msg, the laser scan data.
      -img_size the size of the image.
      -resolution, the resolution of the image.
      -detections, the obtained center of circles in current detection. 
  */

  void publish_poses(const std::shared_ptr<LaserScan> msg,int img_size, double resolution, const std::vector<cv::Vec3f>& detections ){
    PoseArray poses_msg;
    // We want the final array to be in the "odom" frame.
    poses_msg.header.frame_id = "odom"; 
    // Use the scan's timestamp.
    poses_msg.header.stamp = msg->header.stamp; 

    // --- Define Conversion Constants. ---
    const int origin_x = img_size / 2;
    const int origin_y = img_size / 2;
    const std::string lidar_frame = msg->header.frame_id;
    const std::string target_frame = "odom";

    // --- Convert and Transform Each Detected Circle. ---
    for (const auto& circle : detections)
    {
        cv::Point pixel_center(cvRound(circle[0]), cvRound(circle[1]));

        // Step 1: Convert pixel coordinates to LiDAR-frame meters.
        double x_lidar = (pixel_center.x - origin_x) / resolution;
        double y_lidar = (origin_y - pixel_center.y) / resolution; // Note: origin_y - pixel.y

        // Create the pose in the LiDAR's frame.
        geometry_msgs::msg::PoseStamped pose_in_lidar_frame;
        pose_in_lidar_frame.header.stamp = msg->header.stamp;
        pose_in_lidar_frame.header.frame_id = lidar_frame;

        pose_in_lidar_frame.pose.position.x = x_lidar;
        pose_in_lidar_frame.pose.position.y = y_lidar;
        pose_in_lidar_frame.pose.position.z = 0.0; // Assumed 2D having seen the data.

        pose_in_lidar_frame.pose.orientation.w = 1.0; // No rotation.
        pose_in_lidar_frame.pose.orientation.x = 0.0;
        pose_in_lidar_frame.pose.orientation.y = 0.0;
        pose_in_lidar_frame.pose.orientation.z = 0.0;

        // Step 2: Transform the pose from lidar_frame to odom.
        try
        {
            geometry_msgs::msg::PoseStamped pose_in_odom_frame;
            pose_in_odom_frame = tf_buffer_->transform(
                pose_in_lidar_frame, 
                target_frame
            );
            
            // Add the successfully transformed pose to our array.
            poses_msg.poses.push_back(pose_in_odom_frame.pose);
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_WARN(
                this->get_logger(), "Could not transform cylinders pose from %s to %s: %s",
                lidar_frame.c_str(), target_frame.c_str(), ex.what());
        }
    }

    // --- Publish the PoseArray ---
    if (!poses_msg.poses.empty())
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Found and published: " << poses_msg.poses.size() << " cylinders relative to odom frame");
    } else if (!detections.empty()) {
      RCLCPP_INFO(this->get_logger(), "Found cylinders, but failed to transform them.");
    } else {
      RCLCPP_INFO(this->get_logger(), "No cylinders found in this scan.");
    }
    
    // Publish the array (it will be empty if no circles were found or transformed.
    pub_->publish(poses_msg);
  }



  /* 
    This function checks if new detections correspond to new cylinders or previous ones.
    Args; 
      -new_cylinders, this is the detected center of cylinders in current detection.
  */
  void add_detections(std::vector<cv::Point> new_cylinders){

    // Iterating known detection centers.
    for(size_t i = 0; i < cylinders_positions_.size(); i++){
      // Iterating new detections centers.
      for(int j = new_cylinders.size() - 1; j >= 0; j--){
        // Checkin for each new detection if it was already seen in previous detections.
        if(check_proximity(cylinders_positions_[i], new_cylinders[j], 20)){
          // If the center was already seen than it is removed from new detections.
          new_cylinders.erase(new_cylinders.begin() + j);
          RCLCPP_INFO_STREAM(this->get_logger(), "proximity problem!");
        }
      }
    }

    // If some detections where not removed they are inserted into the vector of detections.
    for(size_t j = 0; j < new_cylinders.size(); j++){
      cylinders_positions_.push_back(new_cylinders[j]);
    }

    // Updating the count of detected cylinders variable.
    int new_total_count = static_cast<int>(cylinders_positions_.size());
    detected_cylinders_count_.store(new_total_count, std::memory_order_relaxed);
  }


  rclcpp::Subscription<LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<PoseArray>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_sub_;

  std::vector<cv::Point> cylinders_positions_;
  std::atomic<int> detected_cylinders_count_;
  std::atomic<bool> perception_active_;

  rclcpp::TimerBase::SharedPtr timer_ {
    nullptr
  };
  std::shared_ptr < tf2_ros::TransformListener > tf_listener_ {
    nullptr
  };
  std::unique_ptr < tf2_ros::Buffer > tf_buffer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}