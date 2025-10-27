#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>
#include "gazebo_ros_link_attacher/gazebo_ros_link_attacher.hpp"
#include "gazebo_ros_link_attacher/srv/attach.hpp"
#include <ignition/math/Pose3.hh>

namespace gazebo
{
  GZ_REGISTER_WORLD_PLUGIN(GazeboRosLinkAttacher)

  GazeboRosLinkAttacher::GazeboRosLinkAttacher() : WorldPlugin()
  {
  }

  GazeboRosLinkAttacher::~GazeboRosLinkAttacher()
  {
  }

  void GazeboRosLinkAttacher::Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/)
  {
    // Initialize ROS node
    this->ros_node_ = gazebo_ros::Node::Get();

    if (!this->ros_node_)
    {
      RCLCPP_FATAL(rclcpp::get_logger("GazeboRosLinkAttacher"), "A ROS node for Gazebo has not been initialized.");
      return;
    }

    this->world = _world;
    this->physics = this->world->Physics();

    // Create services
    this->attach_service_ = this->ros_node_->create_service<gazebo_ros_link_attacher::srv::Attach>(
      "attach", 
      std::bind(&GazeboRosLinkAttacher::attach_callback, this, 
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    this->detach_service_ = this->ros_node_->create_service<gazebo_ros_link_attacher::srv::Attach>(
      "detach", 
      std::bind(&GazeboRosLinkAttacher::detach_callback, this, 
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    
    RCLCPP_INFO(this->ros_node_->get_logger(), "Link attacher node initialized.");
  }

  bool GazeboRosLinkAttacher::attach(const std::string &model1, const std::string &link1,
                                     const std::string &model2, const std::string &link2)
  {
    // Look for any previous instance of the joint
    FixedJoint j;
    if(this->getJoint(model1, link1, model2, link2, j)) {
        RCLCPP_INFO(this->ros_node_->get_logger(), "Joint already existed, reusing it.");
        j.joint->Attach(j.l1, j.l2);
        return true;
    } else {
        RCLCPP_INFO(this->ros_node_->get_logger(), "Creating new joint.");
    }

    j.model1 = model1;
    j.link1 = link1;
    j.model2 = model2;
    j.link2 = link2;

    RCLCPP_DEBUG(rclcpp::get_logger("GazeboRosLinkAttacher"), "Getting BasePtr of %s", model1.c_str());
    physics::BasePtr b1 = this->world->ModelByName(model1);
    if (b1 == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("GazeboRosLinkAttacher"), "%s model was not found", model1.c_str());
        return false;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("GazeboRosLinkAttacher"), "Getting BasePtr of %s", model2.c_str());
    physics::BasePtr b2 = this->world->ModelByName(model2);
    if (b2 == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("GazeboRosLinkAttacher"), "%s model was not found", model2.c_str());
        return false;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("GazeboRosLinkAttacher"), "Casting into ModelPtr");
    physics::ModelPtr m1(dynamic_cast<physics::Model*>(b1.get()));
    j.m1 = m1;
    physics::ModelPtr m2(dynamic_cast<physics::Model*>(b2.get()));
    j.m2 = m2;

    RCLCPP_DEBUG(rclcpp::get_logger("GazeboRosLinkAttacher"), "Getting link: '%s' from model: '%s'", 
                 link1.c_str(), model1.c_str());
    physics::LinkPtr l1 = m1->GetLink(link1);
    if (l1 == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("GazeboRosLinkAttacher"), "%s link was not found", link1.c_str());
        return false;
    }
    if (l1->GetInertial() == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("GazeboRosLinkAttacher"), "link1 inertia is NULL!");
    } else {
        RCLCPP_DEBUG(rclcpp::get_logger("GazeboRosLinkAttacher"), 
                     "link1 inertia is not NULL, mass is: %f", l1->GetInertial()->Mass());
    }
    j.l1 = l1;

    RCLCPP_DEBUG(rclcpp::get_logger("GazeboRosLinkAttacher"), "Getting link: '%s' from model: '%s'",
                 link2.c_str(), model2.c_str());
    physics::LinkPtr l2 = m2->GetLink(link2);
    if (l2 == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("GazeboRosLinkAttacher"), "%s link was not found", link2.c_str());
        return false;
    }
    if (l2->GetInertial() == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("GazeboRosLinkAttacher"), "link2 inertia is NULL!");
    } else {
        RCLCPP_DEBUG(rclcpp::get_logger("GazeboRosLinkAttacher"),
                     "link2 inertia is not NULL, mass is: %f", l2->GetInertial()->Mass());
    }
    j.l2 = l2;

    RCLCPP_DEBUG(rclcpp::get_logger("GazeboRosLinkAttacher"), "Creating revolute joint on model: '%s'", model1.c_str());
    j.joint = this->physics->CreateJoint("revolute", m1);
    this->joints.push_back(j);

    j.joint->Attach(l1, l2);
    j.joint->Load(l1, l2, ignition::math::Pose3d());
    j.joint->SetModel(m2);

    j.joint->SetUpperLimit(0, 0);
    j.joint->SetLowerLimit(0, 0);
    j.joint->Init();
    
    RCLCPP_INFO(rclcpp::get_logger("GazeboRosLinkAttacher"), "Attach finished.");
    return true;
}

  bool GazeboRosLinkAttacher::detach(const std::string &model1, const std::string &link1,
                                     const std::string &model2, const std::string &link2)
  {
    FixedJoint j;
    if(this->getJoint(model1, link1, model2, link2, j)) {
        std::lock_guard<std::recursive_mutex> lock(this->physics_mutex_);
        j.joint->Detach();
        RCLCPP_INFO(rclcpp::get_logger("GazeboRosLinkAttacher"), "Detach finished.");
        return true;
    }
    return false;
  }

  bool GazeboRosLinkAttacher::getJoint(const std::string &model1, const std::string &link1,
                                       const std::string &model2, const std::string &link2,
                                       FixedJoint &joint)
  {
    FixedJoint j;
    for(auto it = this->joints.begin(); it != this->joints.end(); ++it) {
        j = *it;
        if ((j.model1.compare(model1) == 0) && (j.model2.compare(model2) == 0)
            && (j.link1.compare(link1) == 0) && (j.link2.compare(link2) == 0)) {
            joint = j;
            return true;
        }
    }
    return false;
  }

  void GazeboRosLinkAttacher::attach_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                              const std::shared_ptr<gazebo_ros_link_attacher::srv::Attach::Request> req,
                                              const std::shared_ptr<gazebo_ros_link_attacher::srv::Attach::Response> res)
  {
    RCLCPP_INFO(rclcpp::get_logger("GazeboRosLinkAttacher"), "Received request to attach model: '%s' using link: '%s' with model: '%s' using link: '%s'",
                 req->model_name_1.c_str(), req->link_name_1.c_str(), req->model_name_2.c_str(), req->link_name_2.c_str());
    res->ok = this->attach(req->model_name_1, req->link_name_1, req->model_name_2, req->link_name_2);
  }

  void GazeboRosLinkAttacher::detach_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                              const std::shared_ptr<gazebo_ros_link_attacher::srv::Attach::Request> req,
                                              const std::shared_ptr<gazebo_ros_link_attacher::srv::Attach::Response> res)
  {
    RCLCPP_INFO(rclcpp::get_logger("GazeboRosLinkAttacher"), "Received request to detach model: '%s' using link: '%s' with model: '%s' using link: '%s'",
                 req->model_name_1.c_str(), req->link_name_1.c_str(), req->model_name_2.c_str(), req->link_name_2.c_str());
    res->ok = this->detach(req->model_name_1, req->link_name_1, req->model_name_2, req->link_name_2);
  }
}