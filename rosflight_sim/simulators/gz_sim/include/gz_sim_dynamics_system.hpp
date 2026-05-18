#ifndef ROSFLIGHT_SIM_GAZEBO_DYNAMICS_SYSTEM_H
#define ROSFLIGHT_SIM_GAZEBO_DYNAMICS_SYSTEM_H

#include <memory>
#include <string>
#include <thread>

#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <rclcpp/rclcpp.hpp>

#include "gz_sim_dynamics.hpp"

// namespace rosflight_sim
// {

class GazeboDynamicsSystem : public gz::sim::System,
                             public gz::sim::ISystemConfigure,
                             public gz::sim::ISystemPreUpdate,
                             public gz::sim::ISystemPostUpdate {
public:
    GazeboDynamicsSystem();

    ~GazeboDynamicsSystem() override;

    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &_eventMgr) override;

    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm) override;

    void PostUpdate(const gz::sim::UpdateInfo &_info,
                    const gz::sim::EntityComponentManager &_ecm) override;

private:
    gz::sim::Model model_{gz::sim::kNullEntity};
    gz::sim::Entity link_entity_{gz::sim::kNullEntity};
    std::string link_name_;
    std::string mav_type_;

    std::shared_ptr<rosflight_sim::GazeboDynamics> gazebo_dynamics_ptr_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spin_thread_;
};

// } // namespace rosflight_sim

#endif
