#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <array>
#include <vector>



#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <unordered_map>


class NavigationNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavigationNode() : Node("navigation_node")
    {
        YAML::Node orders = YAML::LoadFile("./param/orders.yaml");
        YAML::Node positions = YAML::LoadFile("./param/positions.yaml");
       
        this->declare_parameter<std::string>(
            "order",    // se establece el valor que se pasa por terminal con order:="blabla"
            "monday");  // si no se establece monday por defecto
        std::string day_order = this->get_parameter("order").as_string();
        RCLCPP_INFO(this->get_logger(), "Order: %s", day_order.c_str());

        // Lista de días de la semana en inglés
        std::vector<std::string> valid_days = {
            "monday", "tuesday", "wednesday",
            "thursday", "friday", "saturday", "sunday"};
        
        // Comprobar si el día proporcionado es válido
        if (std::find(valid_days.begin(), valid_days.end(), day_order) == valid_days.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid day order: %s. Node will shut down.", day_order.c_str());
            rclcpp::shutdown();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Valid day order: %s", day_order.c_str());
        }

        // ---------------------------------------------------------------------

        for (const auto & order : orders["orders"]["monday"])
        {
            std::string name = order.as<std::string>();
            if (positions["positions"][name])
            {
                float x = positions["positions"][name]["x"].as<float>();
                float y = positions["positions"][name]["y"].as<float>();
                float orientation = positions["positions"][name]["orientation"].as<float>();
                pick_sequence_.emplace_back(name, std::make_tuple(x, y, orientation));
            }
        }
        
        pick_sequence_.emplace_back("end", std::make_tuple(
            positions["positions"]["end"]["x"].as<float>(),
            positions["positions"]["end"]["y"].as<float>(),
            positions["positions"]["end"]["orientation"].as<float>()
        ));

        current_ = pick_sequence_.begin();

        // ---------------------------------------------------------------------

        client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&NavigationNode::try_send_goal, this));
    }

private:
    
    using Pick = std::pair<std::string, std::tuple<float, float, float>>;
    std::vector<Pick> pick_sequence_;
    std::vector<Pick>::iterator current_;

    void try_send_goal()
    {
        if (!client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Esperando al servidor de acciones...");
            return;
        }

        if (goal_active_ || current_ == pick_sequence_.end())
        {
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->get_clock()->now();
        goal_msg.pose.pose.position.x = std::get<0>(current_->second);
        goal_msg.pose.pose.position.y = std::get<1>(current_->second);
        double theta = std::get<2>(current_->second);
        goal_msg.pose.pose.orientation.z = sin(theta / 2.0);
        goal_msg.pose.pose.orientation.w = cos(theta / 2.0);

        RCLCPP_INFO(this->get_logger(), "Enviando meta %ld (%s)...", goal_index_ + 1, current_->first.c_str());

        goal_active_ = true;
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
        options.goal_response_callback =
            std::bind(&NavigationNode::goal_response_callback, this, std::placeholders::_1);
        options.result_callback =
            std::bind(&NavigationNode::result_callback, this, std::placeholders::_1);

        client_->async_send_goal(goal_msg, options);
    }

    void goal_response_callback(GoalHandleNavigateToPose::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Meta rechazada por el servidor.");
            goal_active_ = false;
        }
    }

    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        goal_active_ = false;
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Meta %ld completada con éxito.", goal_index_ + 1);
                goal_index_++;
                ++current_;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_WARN(this->get_logger(), "Meta abortada. Reintentando...");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Meta cancelada. Reintentando...");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Resultado desconocido.");
                break;
        }
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::array<double, 3>> goals_;
    size_t goal_index_ = 0;
    bool goal_active_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationNode>());
    rclcpp::shutdown();
    return 0;
}
