// Continuous, simulator-side contact evidence. Empty frames are explicit observations.
#include <chrono>
#include <vector>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/uint32.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>

namespace robot_validate {
class ContactsMonitor final : public gz::sim::System,
    public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate {
 public:
  ContactsMonitor() {
    contactsPub = node.Advertise<gz::msgs::Contacts>("/contacts");
    countPub = node.Advertise<gz::msgs::UInt32>("/contact_monitor_count");
  }
  void PreUpdate(const gz::sim::UpdateInfo &,
                 gz::sim::EntityComponentManager &ecm) override {
    collisions.clear();
    ecm.Each<gz::sim::components::Collision>(
        [&](const gz::sim::Entity &entity, const gz::sim::components::Collision *) {
      auto parent = entity;
      bool robot = false;
      while (parent != gz::sim::kNullEntity) {
        const auto *name = ecm.Component<gz::sim::components::Name>(parent);
        if (ecm.Component<gz::sim::components::Model>(parent) && name && name->Data() == "robot") {
          robot = true;
          break;
        }
        const auto *relation = ecm.Component<gz::sim::components::ParentEntity>(parent);
        parent = relation ? relation->Data() : gz::sim::kNullEntity;
      }
      if (robot) {
        collisions.push_back(entity);
      }
      return true;
    });
    for (auto entity : collisions) {
      if (!ecm.Component<gz::sim::components::ContactSensorData>(entity))
        ecm.CreateComponent(entity, gz::sim::components::ContactSensorData());
    }
  }
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    unsigned observed = 0;
    for (auto entity : collisions) {
      const auto *data = ecm.Component<gz::sim::components::ContactSensorData>(entity);
      if (!data) continue;
      ++observed;
      for (const auto &contact : data->Data().contact())
        pending.add_contact()->CopyFrom(contact);
    }
    if (info.simTime - last < std::chrono::milliseconds(20)) return;
    last = info.simTime;
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    pending.mutable_header()->mutable_stamp()->set_sec(ns / 1000000000);
    pending.mutable_header()->mutable_stamp()->set_nsec(ns % 1000000000);
    // Count verifies that all four collision geometries were registered, even on empty frames.
    gz::msgs::UInt32 count;
    count.set_data(observed);
    countPub.Publish(count);
    contactsPub.Publish(pending);
    pending.Clear();
  }
 private:
  gz::transport::Node node;
  gz::transport::Node::Publisher contactsPub, countPub;
  std::vector<gz::sim::Entity> collisions;
  gz::msgs::Contacts pending;
  std::chrono::steady_clock::duration last{};
};
}
GZ_ADD_PLUGIN(robot_validate::ContactsMonitor, gz::sim::System,
              robot_validate::ContactsMonitor::ISystemPreUpdate,
              robot_validate::ContactsMonitor::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(robot_validate::ContactsMonitor, "robot_validate::ContactsMonitor")
