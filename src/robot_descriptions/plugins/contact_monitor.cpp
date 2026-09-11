#include <functional>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/bool.hpp>

namespace gazebo
{
class SimContactMonitor : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    node_ = gazebo_ros::Node::Get(sdf);
    publisher_ = node_->create_publisher<std_msgs::msg::Bool>("/eval/contact", 10);
    manager_ = model_->GetWorld()->Physics()->GetContactManager();
    manager_->SetNeverDropContacts(true);
    update_ = event::Events::ConnectWorldUpdateEnd(std::bind(&SimContactMonitor::Update, this));
  }
private:
  void Update()
  {
    const auto & contacts = manager_->GetContacts();
    for (unsigned int i = 0; i < manager_->GetContactCount(); ++i) {
      const auto contact = contacts[i];
      auto first = contact->collision1->GetModel();
      auto second = contact->collision2->GetModel();
      if (first != model_ && second != model_) {continue;}
      auto other = first == model_ ? second : first;
      if (other == model_ || other->GetName() == "ground_plane") {continue;}
      hit_ = true;
    }
    const double now = model_->GetWorld()->SimTime().Double();
    if (now - last_ >= 0.05 || now < last_) {
      std_msgs::msg::Bool message;
      message.data = hit_;
      publisher_->publish(message);
      hit_ = false;
      last_ = now;
    }
  }
  physics::ModelPtr model_;
  physics::ContactManager * manager_{nullptr};
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
  event::ConnectionPtr update_;
  bool hit_{false};
  double last_{0.0};
};
GZ_REGISTER_MODEL_PLUGIN(SimContactMonitor)
}
