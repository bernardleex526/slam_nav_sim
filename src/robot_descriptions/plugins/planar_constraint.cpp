#include <functional>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
// Explicit ideal-planar benchmark model: contacts still constrain x/y, but
// roll, pitch and height are not degrees of freedom of this experiment.
class SimPlanarConstraint : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr) override
  {
    model_ = model;
    height_ = model_->WorldPose().Pos().Z();
    for (const auto & link : model_->GetLinks()) {
      link->SetGravityMode(false);
    }
    update_ = event::Events::ConnectWorldUpdateEnd(
      std::bind(&SimPlanarConstraint::Constrain, this));
  }

private:
  void Constrain()
  {
    auto pose = model_->WorldPose();
    pose.Pos().Z(height_);
    pose.Rot() = ignition::math::Quaterniond(0.0, 0.0, pose.Rot().Yaw());
    auto linear = model_->WorldLinearVel();
    const auto angular = model_->WorldAngularVel();
    linear.Z(0.0);
    model_->SetWorldPose(pose);
    model_->SetLinearVel(linear);
    model_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, angular.Z()));
  }

  physics::ModelPtr model_;
  event::ConnectionPtr update_;
  double height_{0.0};
};

GZ_REGISTER_MODEL_PLUGIN(SimPlanarConstraint)
}  // namespace gazebo
