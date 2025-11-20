#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp" 
#include "nav2_msgs/action/navigate_to_pose.hpp" // Action interface with nav2 library

#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/convert.hpp"
#include "std_msgs/msg/string.hpp"

#include <array>

using namespace std::chrono_literals;

class Nav2ActionClient : public rclcpp::Node
{
public:
    using NavigateToPoseAction = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPoseAction>;

    Nav2ActionClient(const rclcpp::NodeOptions &options)
        : Node("nav2_action_client", options)
    {
        // Init transform buffer and transform listener.
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Client to interface with the action_server bt_navigator
        action_client_ = rclcpp_action::create_client<NavigateToPoseAction>(
            this, "navigate_to_pose");

        // Callback Lambda function that execute the send_goal()
        auto timer_callback_lambda = [this]()
        { return this->send_goal(); };

        // Timer to call the send_goal function, implemented with lambda function 2Hz
        // NOTE : in the implementation below we will see that we call the function just 1 time in reality, so the freq is not so much improtant
        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            timer_callback_lambda);

        start_perception_publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/start_perception",
            10
        );
    }

    void send_goal()
    {
        using namespace std::placeholders;

        // We want to send just one goal in this implementation aka we require just 1 sequence of fibonacci
        this->timer_->cancel();

        // Wait to switch online the fibonacci Server otherwise after a time limit switch off the node
        if (!this->action_client_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        // Define the goal position.
        std::array<double, 2> goal = get_goal_position();

        // Goal service request
        RCLCPP_INFO(this->get_logger(), "Sending goal... ");
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = goal[0];
        goal_msg.pose.pose.position.y = goal[1];
        goal_msg.pose.pose.position.z = 0.0;

        goal_msg.pose.pose.orientation.x = 0.0;
        goal_msg.pose.pose.orientation.y = 0.0;
        goal_msg.pose.pose.orientation.z = 0.0;
        goal_msg.pose.pose.orientation.w = 1.0;
        
        RCLCPP_INFO(this->get_logger(), "Sending goal");

        // Object to embed 3 callbacks:
        auto send_goal_options = rclcpp_action::Client<NavigateToPoseAction>::SendGoalOptions();

        // Goal response callback (Wait the acceptance responce of the Goal by the Server)
        send_goal_options.goal_response_callback = [this](const GoalHandle::SharedPtr &goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        };

        // Feedback callback (wait the feedback from Server)
        send_goal_options.feedback_callback = [this](
                                                  GoalHandle::SharedPtr,
                                                  const std::shared_ptr<const NavigateToPoseAction::Feedback> feedback)
        {
            // Printing the remainin distance from the feedback sended by NavifateToPose action interface
            RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f m", feedback->distance_remaining);
        };

        // Result callbacks (wait for the Result response by the Server)
        send_goal_options.result_callback = [this](const GoalHandle::WrappedResult &result)
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
            {
                std_msgs::msg::String start_msg;
                start_msg.data = "start";
                start_perception_publisher_->publish(start_msg);

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
        this->action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    std::array<double, 2> get_goal_position() {

        auto transform_tag0_base = 
            this->tf_buffer_->lookupTransform(
                base_frame_, // Target frame.
                tag1_frame_, // Source frame.
                tf2::TimePointZero,
                std::chrono::seconds(5) // Wait up to 5 sec.
            );

        auto transform_tag10_base = 
            this->tf_buffer_->lookupTransform(
                base_frame_,  // Target frame.
                tag10_frame_, // Source frame.
                tf2::TimePointZero,
                std::chrono::seconds(5) // Wait up to 5 sec.
            );

        // Point in vc.
        geometry_msgs::msg::PointStamped pt_in_tag1;
        pt_in_tag1.header.frame_id = base_frame_;
        pt_in_tag1.header.stamp = transform_tag0_base.header.stamp;

        // Point in cs.
        geometry_msgs::msg::PointStamped pt_in_tag10;
        pt_in_tag10.header.frame_id = base_frame_;
        pt_in_tag10.header.stamp = transform_tag10_base.header.stamp;

        // Tranform points.
        geometry_msgs::msg::PointStamped v, u;
        tf2::doTransform(pt_in_tag1, v, transform_tag0_base);
        tf2::doTransform(pt_in_tag10, u, transform_tag10_base);


        std::array<double, 2> r;
        r[0] = u.point.x - v.point.x;
        r[1] = u.point.y - v.point.y;

        std::array<double, 2> goal;
        goal[0] = v.point.x + r[0] / 2;
        goal[1] = v.point.y + r[1] / 2;

        goal[0] *= 0.92;
        goal[1] *= 0.92;

        // Log tranformed point.
        RCLCPP_INFO(this->get_logger(),
            "\n\tv  point: x=%.3f y=%.3f z=0"
            "\n\tu point: x=%.3f y=%.3f z=0"
            "\n\tgoal  point: x=%.3f y=%.3f z=0",
            v.point.x, v.point.y,
            u.point.x, u.point.y,
            goal[0], goal[1]
        );

        return goal;
    }

    private:
        rclcpp_action::Client<NavigateToPoseAction>::SharedPtr action_client_; // Action client
        rclcpp::TimerBase::SharedPtr timer_; // Timer for sending the goal (just 1 time)
        rclcpp::TimerBase::SharedPtr timer2_; 
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr start_perception_publisher_; // Publisher for the /start_perception topic

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        const std::string tag1_frame_  = "tag36h11:1";
        const std::string tag10_frame_ = "tag36h11:10";
        const std::string base_frame_  = "base_link";
};

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nav2ActionClient>(rclcpp::NodeOptions())); 
    rclcpp::shutdown();
    return 0;
}

