#include "gz_sim_dynamics.hpp"

#include <gz/math/Matrix3.hh>
#include <gz/math/Quaternion.hh>

#include <gz/sim/components/Component.hh>
// #include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularAcceleration.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/Util.hh>

namespace rosflight_sim {
    GazeboDynamics::GazeboDynamics() : DynamicsInterface() {
    }

    void GazeboDynamics::Configure(gz::sim::Entity model_entity,
                                   gz::sim::Entity link_entity,
                                   const std::string &link_name) {
        std::scoped_lock lock(mutex_);
        model_entity_ = model_entity;
        link_entity_ = link_entity;
        link_name_ = link_name;
        configured_ = (link_entity_ != gz::sim::kNullEntity);
    }

    void GazeboDynamics::apply_forces_and_torques(
        const geometry_msgs::msg::WrenchStamped &forces_torques) {
        std::scoped_lock lock(mutex_);
        pending_wrench_ = forces_torques;
    }

    rosflight_msgs::msg::SimState GazeboDynamics::compute_truth() {
        std::scoped_lock lock(mutex_);
        return latest_truth_;
    }

    geometry_msgs::msg::Vector3Stamped GazeboDynamics::compute_wind_truth() {
        geometry_msgs::msg::Vector3Stamped current_wind;
        current_wind.header.stamp = this->get_clock()->now();
        return current_wind;
    }

    bool GazeboDynamics::set_sim_state(const rosflight_msgs::msg::SimState state) {
        std::scoped_lock lock(mutex_);
        pending_state_ = state;
        std::cout << "set_sim_state() calld." << std::endl;
        return true;
    }


    void GazeboDynamics::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) {
        if (_info.paused)
            return;

        std::optional<geometry_msgs::msg::WrenchStamped> wrench;
        std::optional<rosflight_msgs::msg::SimState> state;
        {
            std::scoped_lock lock(mutex_);
            if (!configured_)
                return;
            wrench = pending_wrench_;
            state = pending_state_;
            pending_wrench_.reset();
            pending_state_.reset();
        }

        gz::sim::Link link(link_entity_);
        if (!link.Valid(_ecm))
            return;

        // Ensures world velocity / pose components are produced by physics.
        link.EnableVelocityChecks(_ecm, true);

        if (wrench.has_value()) {
            // Original plugin used body-relative NWU forces/torques:
            //   (x, -y, -z)
            // Harmonic wrench application is in world coordinates, so rotate the
            // body-frame command into world first.
            const auto worldPose = gz::sim::worldPose(link_entity_, _ecm);
            const auto R = worldPose.Rot();

            const gz::math::Vector3d bodyForce(
                wrench->wrench.force.x,
                -wrench->wrench.force.y,
                -wrench->wrench.force.z);
            const gz::math::Vector3d bodyTorque(
                wrench->wrench.torque.x,
                -wrench->wrench.torque.y,
                -wrench->wrench.torque.z);

            link.AddWorldWrench(_ecm, R.RotateVector(bodyForce), R.RotateVector(bodyTorque));
        }

        if (state.has_value()) {
            // Map state into Gazebo NWU / world-frame commands.
            auto poseCmd = _ecm.Component<gz::sim::v8::components::WorldPoseCmd>(link_entity_);
            const gz::math::Pose3<double> newPose(
                state->pose.position.x,
                -state->pose.position.y,
                -state->pose.position.z,
                state->pose.orientation.w,
                state->pose.orientation.x,
                -state->pose.orientation.y,
                -state->pose.orientation.z);

            if (poseCmd)
                poseCmd->SetData(newPose, [](auto &, const auto &) { return false; });
            else
                _ecm.CreateComponent(link_entity_, gz::sim::v8::components::WorldPoseCmd(newPose));

            link.SetLinearVelocity(_ecm, gz::math::Vector3d(
                                       state->twist.linear.x,
                                       -state->twist.linear.y,
                                       -state->twist.linear.z));

            link.SetAngularVelocity(_ecm, gz::math::Vector3d(
                                        state->twist.angular.x,
                                        -state->twist.angular.y,
                                        -state->twist.angular.z));
        }
    }

    rosflight_msgs::msg::SimState GazeboDynamics::truth_from_ecm_(
        const gz::sim::EntityComponentManager &_ecm) const {
        rosflight_msgs::msg::SimState truth;
        truth.header.stamp = this->now();
        truth.header.frame_id = link_name_ + "_NED";

        // World pose of the link.
        const auto pose = gz::sim::worldPose(link_entity_, _ecm);

        // Rotation from body -> world.
        const auto R_wb = pose.Rot();

        // Rotation from world -> body.
        const auto R_bw = R_wb.Inverse();

        // Read world-frame quantities from Gazebo Sim.
        const auto worldLinVelComp =
                _ecm.Component<gz::sim::components::WorldLinearVelocity>(link_entity_);
        const auto worldAngVelComp =
                _ecm.Component<gz::sim::components::WorldAngularVelocity>(link_entity_);
        const auto worldLinAccComp =
                _ecm.Component<gz::sim::components::WorldLinearAcceleration>(link_entity_);
        const auto worldAngAccComp =
                _ecm.Component<gz::sim::components::WorldAngularAcceleration>(link_entity_);

        // Pose remains an inertial/world quantity.
        // Convert orientation and position from Gazebo NWU to ROSflight NED.
        truth.pose.orientation.w = R_wb.W();
        truth.pose.orientation.x = R_wb.X();
        truth.pose.orientation.y = -R_wb.Y();
        truth.pose.orientation.z = -R_wb.Z();

        truth.pose.position.x = pose.Pos().X();
        truth.pose.position.y = -pose.Pos().Y();
        truth.pose.position.z = -pose.Pos().Z();

        // Convert world linear velocity -> body linear velocity -> NED body axes.
        if (worldLinVelComp) {
            const auto &v_world = worldLinVelComp->Data();
            const gz::math::Vector3d v_body = R_bw.RotateVector(v_world);

            truth.twist.linear.x = v_body.X();
            truth.twist.linear.y = -v_body.Y();
            truth.twist.linear.z = -v_body.Z();
        } else {
            truth.twist.linear.x = 0.0;
            truth.twist.linear.y = 0.0;
            truth.twist.linear.z = 0.0;
        }

        // Convert world angular velocity -> body angular velocity -> NED body axes.
        if (worldAngVelComp) {
            const auto &w_world = worldAngVelComp->Data();
            const gz::math::Vector3d w_body = R_bw.RotateVector(w_world);

            truth.twist.angular.x = w_body.X();
            truth.twist.angular.y = -w_body.Y();
            truth.twist.angular.z = -w_body.Z();
        } else {
            truth.twist.angular.x = 0.0;
            truth.twist.angular.y = 0.0;
            truth.twist.angular.z = 0.0;
        }

        // Convert world linear acceleration -> body linear acceleration -> NED body axes.
        if (worldLinAccComp) {
            const auto &a_world = worldLinAccComp->Data();
            const gz::math::Vector3d a_body = R_bw.RotateVector(a_world);

            truth.acceleration.linear.x = a_body.X();
            truth.acceleration.linear.y = -a_body.Y();
            truth.acceleration.linear.z = -a_body.Z();
        } else {
            truth.acceleration.linear.x = 0.0;
            truth.acceleration.linear.y = 0.0;
            truth.acceleration.linear.z = 0.0;
        }

        // Convert world angular acceleration -> body angular acceleration -> NED body axes.
        if (worldAngAccComp) {
            const auto &aa_world = worldAngAccComp->Data();
            const gz::math::Vector3d aa_body = R_bw.RotateVector(aa_world);

            truth.acceleration.angular.x = aa_body.X();
            truth.acceleration.angular.y = -aa_body.Y();
            truth.acceleration.angular.z = -aa_body.Z();
        } else {
            truth.acceleration.angular.x = 0.0;
            truth.acceleration.angular.y = 0.0;
            truth.acceleration.angular.z = 0.0;
        }

        return truth;
    }

    void GazeboDynamics::PostUpdate(const gz::sim::UpdateInfo &_info,
                                    const gz::sim::EntityComponentManager &_ecm) {
        if (_info.paused)
            return;

        std::scoped_lock lock(mutex_);
        if (!configured_)
            return;
        latest_truth_ = truth_from_ecm_(_ecm);
    }
} // namespace rosflight_sim
