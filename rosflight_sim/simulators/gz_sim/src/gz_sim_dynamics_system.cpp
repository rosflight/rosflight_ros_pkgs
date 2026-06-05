#include "gz_sim_dynamics_system.hpp"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>

// namespace rosflight_sim {
GazeboDynamicsSystem::GazeboDynamicsSystem() = default;

GazeboDynamicsSystem::~GazeboDynamicsSystem() {
    if (executor_ && gazebo_dynamics_ptr_)
        executor_->remove_node(gazebo_dynamics_ptr_);

    if (rclcpp::ok())
        rclcpp::shutdown();

    if (spin_thread_.joinable())
        spin_thread_.join();
}

void GazeboDynamicsSystem::Configure(const gz::sim::Entity &_entity,
                                     const std::shared_ptr<const sdf::Element> &_sdf,
                                     gz::sim::EntityComponentManager &_ecm,
                                     gz::sim::EventManager &) {
    if (!rclcpp::ok()) {
        int argc = 0;
        char **argv = nullptr;
        rclcpp::init(argc, argv);
    }

    model_ = gz::sim::Model(_entity);
    if (!model_.Valid(_ecm)) {
        gzerr << "[rosflight_sim] GazeboDynamicsSystem must be attached to a model.\n";
        return;
    }

    if (_sdf && _sdf->HasElement("link_name"))
        link_name_ = _sdf->Get<std::string>("link_name");
    else if (_sdf && _sdf->HasElement("linkName"))
        link_name_ = _sdf->Get<std::string>("linkName");

    if (link_name_.empty()) {
        link_entity_ = model_.CanonicalLink(_ecm);
    } else {
        link_entity_ = model_.LinkByName(_ecm, link_name_);
    }

    if (_sdf && _sdf->HasElement("mav_type"))
        mav_type_ = _sdf->Get<std::string>("mav_type");
    else if (_sdf && _sdf->HasElement("mavType"))
        mav_type_ = _sdf->Get<std::string>("mavType");
    else
        mav_type_ = "multirotor";

    if (link_entity_ == gz::sim::kNullEntity) {
        gzerr << "[rosflight_sim] Could not resolve dynamics link for model ["
                << model_.Name(_ecm) << "]\n";
        return;
    }

    gazebo_dynamics_ptr_ = std::make_shared<rosflight_sim::GazeboDynamics>();
    gazebo_dynamics_ptr_->Configure(_entity, link_entity_,
                                    link_name_.empty() ? "canonical_link" : link_name_);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(gazebo_dynamics_ptr_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
}

void GazeboDynamicsSystem::PreUpdate(const gz::sim::UpdateInfo &_info,
                                     gz::sim::EntityComponentManager &_ecm) {
    if (gazebo_dynamics_ptr_)
        gazebo_dynamics_ptr_->PreUpdate(_info, _ecm);
}

void GazeboDynamicsSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
                                      const gz::sim::EntityComponentManager &_ecm) {
    if (gazebo_dynamics_ptr_)
        gazebo_dynamics_ptr_->PostUpdate(_info, _ecm);
}

// } // namespace rosflight_sim

GZ_ADD_PLUGIN(
    GazeboDynamicsSystem,
    gz::sim::System,
    GazeboDynamicsSystem::ISystemConfigure,
    GazeboDynamicsSystem::ISystemPreUpdate,
    GazeboDynamicsSystem::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(GazeboDynamicsSystem, "rosflight_sim::GazeboDynamicsSystem")
