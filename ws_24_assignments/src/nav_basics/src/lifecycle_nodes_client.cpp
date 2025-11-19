#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp" // Msg needed for the Topic
#include <array> // Necessario per popolare l'array di covarianz


class ManageLifecycleNodesClient : public rclcpp::Node
{
public:
    ManageLifecycleNodesClient() : Node("lifecycle_nodes_client")
    {

        // Client of navigation and localization
        navigation_client_ = create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_navigation/manage_nodes");
        
        localization_client_ = create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_localization/manage_nodes");
        

        // Publisher for setting the initial pose
        initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose",
            10
        );
    }

    void send_request_localization()
    {

        
        auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
        // Set request->command as needed
        request->command = 0; // startup nav2 

        RCLCPP_INFO(get_logger(), "Wait for service of localization");
        localization_client_->wait_for_service();
        auto result_future = localization_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Service call of localization completed");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Service call of localization failed");
        }

        // Writing on the topic the initial position of the robot
        publish_initial_pose();
    }

    void send_request_navigation(){

        auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
        // Set request->command as needed
        request->command = 0; // startup nav2 

        RCLCPP_INFO(get_logger(), "Wait for service of navigation");
        navigation_client_->wait_for_service();
        
        auto result_future = navigation_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Service call of navigation completed");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Service call of navigation failed");
        }
    }
   
          void publish_initial_pose()
        {

            // 3. Creazione e Popolazione del Messaggio
            auto initial_pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();

            // --- HEADER ---
            // frame_id: 'map'
            initial_pose_msg.header.frame_id = "map";
            initial_pose_msg.header.stamp = this->now();

            // --- POSE: POSITION ---
            // position: {x: 0.0, y: 0.0, z: 0.0}
            initial_pose_msg.pose.pose.position.x = 0.0;
            initial_pose_msg.pose.pose.position.y = 0.0;
            initial_pose_msg.pose.pose.position.z = 0.0;

            // --- POSE: ORIENTATION (Quaternion) ---
            // orientation: {z: 0.0, w: 1.0} (Orientamento nullo / nessuna rotazione, z=0.0, w=1.0)
            initial_pose_msg.pose.pose.orientation.x = 0.0;
            initial_pose_msg.pose.pose.orientation.y = 0.0;
            initial_pose_msg.pose.pose.orientation.z = 0.0;
            initial_pose_msg.pose.pose.orientation.w = 1.0;

            // --- COVARIANCE (Array 6x6) ---
            // I valori di covarianza devono essere inseriti in un array di 36 elementi (6x6)
            // I valori che hai fornito indicano un'incertezza sulla posizione (x, y) e sull'orientamento (theta / z-rotation)
            std::array<double, 36> covariance_array = {
                // Row 1 (x-x, x-y, x-z, x-roll, x-pitch, x-yaw)
                0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                // Row 2 (y-x, y-y, y-z, y-roll, y-pitch, y-yaw)
                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                // Row 3 (z-x, z-y, z-z, z-roll, z-pitch, z-yaw)
                0.0, 0.0, 0.068, 0.0, 0.0, 0.0, // Nota: Z è la terza riga
                // Row 4 (roll-x, roll-y, roll-z, roll-roll, roll-pitch, roll-yaw)
                0.0, 0.0, 0.0, 0.068, 0.0, 0.0,
                // Row 5 (pitch-x, pitch-y, pitch-z, pitch-roll, pitch-pitch, pitch-yaw)
                0.0, 0.0, 0.0, 0.0, 0.068, 0.0,
                // Row 6 (yaw-x, yaw-y, yaw-z, yaw-roll, yaw-pitch, yaw-yaw)
                0.0, 0.0, 0.0, 0.0, 0.0, 0.068};
            initial_pose_msg.pose.covariance = covariance_array;

            // 4. Pubblicazione del Messaggio
            initial_pose_publisher_->publish(initial_pose_msg);
            RCLCPP_INFO(this->get_logger(), "Published Initial Pose at (0.0, 0.0) in 'map' frame.");
            
        }








private:
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr navigation_client_;
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr localization_client_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_; // Publisher for the /initialpose topic
    
};



// Main function
int main(int argc, char **argv)
{
    // Intialize ros2 cpp library
    rclcpp::init(argc, argv);

    // Create an instance of the burrow node
    std::shared_ptr<ManageLifecycleNodesClient> client_node = std::make_shared<ManageLifecycleNodesClient>();
        
    client_node->send_request_localization();
    client_node->send_request_navigation();
    

    rclcpp::shutdown();
    return 0;
}