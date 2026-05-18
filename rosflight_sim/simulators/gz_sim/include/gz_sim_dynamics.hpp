#ifndef ROSFLIGHT_SIM_GAZEBO_DYNAMICS_H
#define ROSFLIGHT_SIM_GAZEBO_DYNAMICS_H

#include <mutex>
#include <optional>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
// #include <gz/sim/UpdateInfo.hh>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rosflight_msgs/msg/sim_state.hpp"
#include "rosflight_sim/dynamics_interface.hpp"

namespace rosflight_sim {
    class GazeboDynamics : public DynamicsInterface {
    public:
        GazeboDynamics();

        void Configure(gz::sim::Entity model_entity,
                       gz::sim::Entity link_entity,
                       const std::string &link_name);

        void PreUpdate(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm);

        void PostUpdate(const gz::sim::UpdateInfo &_info,
                        const gz::sim::EntityComponentManager &_ecm);

    private:
        void apply_forces_and_torques(const geometry_msgs::msg::WrenchStamped &forces_torques) override;

        rosflight_msgs::msg::SimState compute_truth() override;

        geometry_msgs::msg::Vector3Stamped compute_wind_truth() override;

        bool set_sim_state(const rosflight_msgs::msg::SimState state) override;

        rosflight_msgs::msg::SimState truth_from_ecm_(const gz::sim::EntityComponentManager &_ecm) const;

        mutable std::mutex mutex_;

        bool configured_{false};
        std::string link_name_;
        gz::sim::Entity model_entity_{gz::sim::kNullEntity};
        gz::sim::Entity link_entity_{gz::sim::kNullEntity};

        std::optional<geometry_msgs::msg::WrenchStamped> pending_wrench_;
        std::optional<rosflight_msgs::msg::SimState> pending_state_;
        rosflight_msgs::msg::SimState latest_truth_;

    };
} // namespace rosflight_sim

#endif
