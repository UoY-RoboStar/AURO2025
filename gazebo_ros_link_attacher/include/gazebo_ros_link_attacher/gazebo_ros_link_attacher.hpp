#ifndef GAZEBO_ROS2_LINK_ATTACHER_HPP
#define GAZEBO_ROS2_LINK_ATTACHER_HPP

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>

#include "gazebo_ros_link_attacher/srv/attach.hpp"

namespace gazebo
{
class GazeboRosLinkAttacher : public WorldPlugin
{
public:
    /// \brief Constructor
    GazeboRosLinkAttacher();

    /// \brief Destructor
    virtual ~GazeboRosLinkAttacher();

    /// \brief Load the plugin
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

private:
    /// \brief Internal representation of a fixed joint
    struct FixedJoint {
        std::string model1;
        physics::ModelPtr m1;
        std::string link1;
        physics::LinkPtr l1;
        std::string model2;
        physics::ModelPtr m2;
        std::string link2;
        physics::LinkPtr l2;
        physics::JointPtr joint;
    };

    /// \brief Attach with a revolute joint
    bool attach(const std::string &model1, const std::string &link1,
                const std::string &model2, const std::string &link2);

    /// \brief Detach joint
    bool detach(const std::string &model1, const std::string &link1,
                const std::string &model2, const std::string &link2);

    /// \brief Get existing joint
    bool getJoint(const std::string &model1, const std::string &link1,
                  const std::string &model2, const std::string &link2,
                  FixedJoint &joint);

    /// \brief Callback for attach service
    void attach_callback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<gazebo_ros_link_attacher::srv::Attach::Request> req,
        const std::shared_ptr<gazebo_ros_link_attacher::srv::Attach::Response> res);

    /// \brief Callback for detach service
    void detach_callback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<gazebo_ros_link_attacher::srv::Attach::Request> req,
        const std::shared_ptr<gazebo_ros_link_attacher::srv::Attach::Response> res);

    /// \brief Node for ROS communication
    std::shared_ptr<gazebo_ros::Node> ros_node_;
    
    /// \brief Services
    rclcpp::Service<gazebo_ros_link_attacher::srv::Attach>::SharedPtr attach_service_;
    rclcpp::Service<gazebo_ros_link_attacher::srv::Attach>::SharedPtr detach_service_;

    /// \brief Mutex for thread safety
    std::recursive_mutex physics_mutex_;

    /// \brief Vector to store joints
    std::vector<FixedJoint> joints;

    /// \brief Pointer to the physics engine
    physics::PhysicsEnginePtr physics;

    /// \brief Pointer to the world
    physics::WorldPtr world;
};

}  // namespace gazebo

#endif  // GAZEBO_ROS2_LINK_ATTACHER_HPP