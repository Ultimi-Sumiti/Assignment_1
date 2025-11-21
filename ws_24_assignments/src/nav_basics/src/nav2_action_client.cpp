#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp" 
#include "nav2_msgs/action/navigate_to_pose.hpp" // Action interface with nav2 library

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "std_msgs/msg/string.hpp"

#include <array>

using namespace std::chrono_literals;

// Class used to comunicate with Nav2 Action Server.
class Nav2ActionClient : public rclcpp::Node
{
 public:
    using NavigateToPoseAction = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPoseAction>;

    Nav2ActionClient(const rclcpp::NodeOptions &options)
        : Node("nav2_action_client", options),
          tag1_frame_("tag36h11:1"),
          tag10_frame_("tag36h11:10"),
          base_frame_("base_link")
    {
        // Init transform buffer and transform listener.
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Used to interface with the action_server bt_navigator
        action_client_ = rclcpp_action::create_client<NavigateToPoseAction>(
            this, "navigate_to_pose");

        // Used to execute send_goal() function.
        auto send_goal_lambda = [this]() { return this->send_goal(); };
        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            send_goal_lambda);

        // Create the publisher that will start the detection process.
        percept_publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/start_perception", 10);
    }

    // Function that creates the action goal.
    void send_goal()
    {
        // Wait action server.
        if (!this->action_client_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(),
                "Action server not available after waiting");
            return;
        }

        // Define goal position.
        set_goal_position();

        // Manage the case in which /tf is not available.
        if (goal_position_[0] == 0 && goal_position_[1] == 0)
            return;

        // Stop timer, just one goal is sent.
        this->timer_->cancel();

        // Create the goal request.
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();

        // Position.
        goal_msg.pose.pose.position.x = goal_position_[0];
        goal_msg.pose.pose.position.y = goal_position_[1];
        goal_msg.pose.pose.position.z = 0.0;

        // Orientation.
        goal_msg.pose.pose.orientation.x = 0.0;
        goal_msg.pose.pose.orientation.y = 0.0;
        goal_msg.pose.pose.orientation.z = 0.0;
        goal_msg.pose.pose.orientation.w = 1.0;
        
        // Define the goal options.
        auto goal_options = rclcpp_action::Client<NavigateToPoseAction>::SendGoalOptions();

        // Response callback.
        goal_options.goal_response_callback = 
            [this](const GoalHandle::SharedPtr &goal_handle) {
                if (!goal_handle)
                    RCLCPP_ERROR(this->get_logger(),
                        "Goal was rejected by server");
                else
                    RCLCPP_INFO(this->get_logger(),
                        "Goal accepted by server, waiting for result");
            };

        // Feedback callback.
        goal_options.feedback_callback = 
            [this](
                GoalHandle::SharedPtr,
                const std::shared_ptr<const NavigateToPoseAction::Feedback> feedback
            ) {
                RCLCPP_INFO(this->get_logger(), 
                    "Distance remaining: %.2f m", feedback->distance_remaining);
            };

        // Result callback.
        goal_options.result_callback =
            [this](const GoalHandle::WrappedResult &result) {
                switch (result.code)
                {
                case rclcpp_action::ResultCode::SUCCEEDED:
                {
                    // Notify perception node!
                    std_msgs::msg::String start_msg;
                    start_msg.data = "start";
                    percept_publisher_->publish(start_msg);

                    RCLCPP_INFO(this->get_logger(), "Goal has succeded");
                    break;
                }
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                    return;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    return;
                }
                rclcpp::shutdown();
            };

        RCLCPP_INFO(this->get_logger(), "Sending goal...");
        this->action_client_->async_send_goal(goal_msg, goal_options);
    }

    void set_goal_position() {
        // Transform: tag1 -> base link.
        geometry_msgs::msg::TransformStamped T_tag1_base;
        // Transform: tag10 -> base link.
        geometry_msgs::msg::TransformStamped T_tag10_base;

        // Init goal position to zero.
        goal_position_[0] = 0.0;
        goal_position_[1] = 0.0;

        try {
            T_tag1_base = this->tf_buffer_->lookupTransform(
                base_frame_, // Target frame.
                tag1_frame_, // Source frame.
                tf2::TimePointZero,
                std::chrono::seconds(5) // Wait up to 5 sec.
            );

            T_tag10_base = this->tf_buffer_->lookupTransform(
                base_frame_,  // Target frame.
                tag10_frame_, // Source frame.
                tf2::TimePointZero,
                std::chrono::seconds(5) // Wait up to 5 sec.
            );

        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "No TRANSFORM");
            return;
        }

        // Save tag1 and tag10 position w.r.t. base link.
        double tag1[2], tag10[2];
        tag1[0] = T_tag1_base.transform.translation.x;
        tag1[1] = T_tag1_base.transform.translation.y;
        tag10[0] = T_tag10_base.transform.translation.x;
        tag10[1] = T_tag10_base.transform.translation.y;

        // Compute distance vector between tag1 -> tag10.
        double dist[2];
        dist[0] = tag10[0] - tag1[0];
        dist[1] = tag10[1] - tag1[1];

        // Finally set the goal position.
        goal_position_[0] = tag1[0] + dist[0] / 2;
        goal_position_[1] = tag1[1] + dist[1] / 2;

        // Scale goal position.
        goal_position_[0] *= 0.92;
        goal_position_[1] *= 0.92;

        // Log tranformed point.
        RCLCPP_INFO(this->get_logger(),
            "Goal position: x=%.3f y=%.3f z=0",
            goal_position_[0], goal_position_[1]
        );
    }

    private:
        // Used to create an action goal.
        rclcpp_action::Client<NavigateToPoseAction>::SharedPtr action_client_;
        
        // Used to notify perception node.
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr percept_publisher_;

        // Timer used to call send_goal() function.
        rclcpp::TimerBase::SharedPtr timer_;

        // Used to get position of tags w.r.t. base frame.
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // Name of the frames.
        const std::string tag1_frame_;
        const std::string tag10_frame_;
        const std::string base_frame_;

        // The goal position sended to nav2 action server.
        std::array<double, 2> goal_position_;
};


// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nav2ActionClient>(rclcpp::NodeOptions())); 
    rclcpp::shutdown();
    return 0;
}

