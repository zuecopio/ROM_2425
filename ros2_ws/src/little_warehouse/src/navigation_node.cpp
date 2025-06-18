/**
 * @file    navigation_node.cpp
 *
 * @brief   ROS2 node sends navigation goals from YAML config.
 *
 * This ROS2 node reads navigation orders and corresponding positions from YAML
 * configuration files, then sequentially sends navigation goals to the
 * "navigate_to_pose" action server.
 *
 * The node allows selecting an order day via a parameter (defaulting to
 * "monday"), validates it, and navigates the robot through the specified
 * waypoints ending at a defined "end" position.
 *
 * It handles action server communication, goal sending, and result feedback
 * including success, abort, and cancellation.
 *
 * @authors Marcos Belda Martinez <mbelmar@etsinf.upv.es>,
 *          Angela Espert Cornejo <aespcor@etsinf.upv.es>,
 *          Lourdes Frances Limera <lfralli@epsa.upv.es>
 *
 * @date    June 2025
 */

//-----[ NECESSARY LIBRARIES ]------------------------------------------------//

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <array>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <string>


//-----[ NAVIGATION NODE CLASS ]----------------------------------------------//

/**
 * @class NavigationNode
 * @brief ROS 2 node that handles robot navigation based on daily task orders.
 *
 * This node:
 * - Loads task orders and positions from YAML configuration files.
 * - Accepts a parameter "order" to select the day of the week.
 * - Validates the provided day and builds a navigation sequence.
 * - Sends navigation goals to the robot using the NavigateToPose action client.
 */
class NavigationNode : public rclcpp::Node
{
    public:

        /*-----------------------------------------------------+
         | DECLARATION OF PUBLIC VARIABLES & FUNCTIONS         |
         +-----------------------------------------------------*/
        
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

        /**********************************************************************/
        /*!
         * @brief  NavigationNode class constructor implementation.
         */
        NavigationNode() : Node("navigation_node")
        {
            // Load orders and positions from YAML files
            YAML::Node orders = YAML::LoadFile("./params/orders.yaml");
            YAML::Node positions = YAML::LoadFile("./params/positions.yaml");

            // Declare a parameter "order" which can be set from the terminal,
            // defaulting to "monday" if not provided
            this->declare_parameter<std::string>(
                "order",    // The name of the parameter (set via terminal as
                            // order:=<value>)
                "monday");  // Default value if none is provided

            std::string day_order = this->get_parameter("order").as_string();
            RCLCPP_INFO(this->get_logger(), "Order: %s", day_order.c_str());

            // List of valid weekdays
            std::vector<std::string> valid_days = {
                "monday", "tuesday", "wednesday",
                "thursday", "friday", "saturday", "sunday"
            };

            // Check if the provided day is valid
            if (std::find(valid_days.begin(), valid_days.end(), day_order)
                == valid_days.end())
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Invalid day order: %s. Node will shut down.",
                    day_order.c_str()
                );
                rclcpp::shutdown();
            }
            else
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Valid day order: %s",
                    day_order.c_str()
                );
            }

            // Populate the pick sequence from the orders and positions YAML
            // files for the chosen day
            for (const auto & order : orders["orders"]["monday"])
            {
                std::string name = order.as<std::string>();
                if (positions["positions"][name])
                {
                    float x = positions["positions"][name]["x"].as<float>();
                    float y = positions["positions"][name]["y"].as<float>();
                    float orientation = (
                        positions["positions"][name]["orientation"].as<float>());
                        
                    if (last_x != x)
                    {
                    	// añadir pivote de end de la fila (depende de last_x)
                    	// y añadir el pivote de start de la siguiente fila depende de (x)
                    	// actualizar last_x
                    	
                    }

                    pick_sequence_.emplace_back(
                        name, std::make_tuple(x, y, orientation));
                }
            }

            // Add an "end" position at the end of the pick sequence
            pick_sequence_.emplace_back("end", std::make_tuple(
                positions["positions"]["end"]["x"].as<float>(),
                positions["positions"]["end"]["y"].as<float>(),
                positions["positions"]["end"]["orientation"].as<float>()
            ));

            current_ = pick_sequence_.begin();

            // Create the action client to send navigation goals
            client_ = rclcpp_action::create_client<NavigateToPose>(
                this, "navigate_to_pose");

            // Timer to periodically try sending the next goal
            timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&NavigationNode::try_send_goal, this)
            );

        }   /* NavigationNode() */

    private:

        /*-----------------------------------------------------+
         | DECLARATION OF PRIVATE VARIABLES & FUNCTIONS        |
         +-----------------------------------------------------*/    
        
        /// Variables

        using Pick = std::pair<std::string, std::tuple<float, float, float>>;
        std::vector<Pick> pick_sequence_;
        std::vector<Pick>::iterator current_;

        rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<std::array<double, 3>> goals_;
        size_t goal_index_ = 0;
        bool goal_active_ = false;

        /// Functions

        /**********************************************************************/
        /*!
         * @brief  Try to send the next navigation goal if the server is
         *         available and no goal is active.
         * @param  void
         * @return void
         */
        void try_send_goal(void)
        {
            if (!client_->wait_for_action_server(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Waiting for the action server...");
                return;
            }

            if (goal_active_ || current_ == pick_sequence_.end())
            {
                return;
            }

            // Prepare the goal message with the current target pose
            auto goal_msg = NavigateToPose::Goal();
            goal_msg.pose.header.frame_id = "map";
            goal_msg.pose.header.stamp = this->get_clock()->now();
            goal_msg.pose.pose.position.x = std::get<0>(current_->second);
            goal_msg.pose.pose.position.y = std::get<1>(current_->second);
            double theta = std::get<2>(current_->second);
            goal_msg.pose.pose.orientation.z = sin(theta / 2.0);
            goal_msg.pose.pose.orientation.w = cos(theta / 2.0);

            RCLCPP_INFO(
                this->get_logger(),
                "Sending goal %ld (%s)...",
                goal_index_ + 1, current_->first.c_str()
            );

            goal_active_ = true;
            rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
            
            options.goal_response_callback =
                std::bind(
                    &NavigationNode::goal_response_callback,
                    this, std::placeholders::_1);
            
                options.result_callback =
                std::bind(
                    &NavigationNode::result_callback,
                    this, std::placeholders::_1);

            client_->async_send_goal(goal_msg, options);
        
        }   /* try_send_goal() */
    
        /**********************************************************************/
        /*!
         * @brief  Called when the server accepts or rejects the goal.
         * @param  goal_handle  Handle to the goal, or nullptr if rejected.
         * @return void
         */
        void goal_response_callback(
            GoalHandleNavigateToPose::SharedPtr goal_handle)
        {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal rejected by server.");
                goal_active_ = false;
            }
        
        }   /* goal_response_callback() */

        /**********************************************************************/
        /*!
         * @brief  Called when the goal finishes (success, abort, cancel).
         * @param  result  Result of the goal execution.
         * @return void
         */
        void result_callback(
            const GoalHandleNavigateToPose::WrappedResult & result)
        {
            goal_active_ = false;

            switch (result.code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Goal %ld succeeded.",
                        goal_index_ + 1
                    );
                    goal_index_++;
                    ++current_;
                    break;

                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Goal aborted. Retrying..."
                    );
                    break;

                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Goal canceled. Retrying..."
                    );
                    break;

                default:
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Unknown result code."
                    );
                    break;
            }
        
        }   /* result_callback() */
};


//-----[ MAIN FUNCTION ]------------------------------------------------------//

/**
 * @brief Entry point of the program. Initializes ROS 2, spins the
 *        NavigationNode, and shuts down cleanly.
 * @param  argc  Number of command-line arguments.
 * @param  argv  of command-line argument strings.
 * @return Exit status code (0 for success).
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationNode>());
    rclcpp::shutdown();
    return (0);

}   /* main() */

/*** end of file ***/
