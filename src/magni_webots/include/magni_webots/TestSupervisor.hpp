#ifndef MAGNI_WEBOTS_TEST_SUPERVISOR_HPP
#define MAGNI_WEBOTS_TEST_SUPERVISOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <string>
#include <unordered_map>
#include <vector>

namespace magni_webots {
    class TestSupervisor : public webots_ros2_driver::PluginInterface {
        public:
            TestSupervisor();
            void step() override;
            void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
        private:
            struct TrackedNode {
                std::string name;
                WbNodeRef node_ref;
                rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
            };

            rclcpp::Logger logger_;
            webots_ros2_driver::WebotsNode *node_;
            std::vector<TrackedNode> tracked_nodes_;
            WbNodeRef find_node_by_name(const std::string &name);
            void publish_pose(TrackedNode &tracked);
    };
}
#endif // MAGNI_WEBOTS_TEST_SUPERVISOR_HPP
