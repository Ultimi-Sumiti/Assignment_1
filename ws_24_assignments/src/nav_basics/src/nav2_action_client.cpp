#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp" // Msg needed for the Topic
#include <array> // Necessario per popolare l'array di covarianza

//#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;



class Nav2ActionClient : public rclcpp::Node
{
public:
    using NavigateToPoseAction = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPoseAction>;

    Nav2ActionClient(const rclcpp::NodeOptions &options)
        : Node("nav2_action_client", options)
    {

        action_client_ = rclcpp_action::create_client<NavigateToPoseAction>(
            this, "navigate_to_pose");

        // Callback Lambda function that execute the send_goal()
        auto timer_callback_lambda = [this]()
        { return this->send_goal(); };

        // Timer to call the send_goal function, implemented with lambda function 2Hz
        // NOTE : in the implementation below we will see that we call the function just 1 time in reality so the freq is not so much improtant
        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            timer_callback_lambda);
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

        RCLCPP_INFO(this->get_logger(), "Sending goal... ");

        // Goal service request
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = 10.0;
        goal_msg.pose.pose.position.y = 1.0;
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
            // Puoi stampare la distanza rimanente, che è un dato utile nel feedback di NavigateToPose
            RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f m", feedback->distance_remaining);
        };

        // Result callbacks (wait for the Result response by the Server)
        send_goal_options.result_callback = [this](const GoalHandle::WrappedResult &result)
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal has succeded");
                break;
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

    private:
        rclcpp_action::Client<NavigateToPoseAction>::SharedPtr action_client_;

        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_; // Publisher for the /initialpose topic
        rclcpp::TimerBase::SharedPtr timer_;
        

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Usa std::make_shared con NodeOptions per il costruttore corretto
    rclcpp::spin(std::make_shared<Nav2ActionClient>(rclcpp::NodeOptions())); 
    rclcpp::shutdown();
    return 0;
}

