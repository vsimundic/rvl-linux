#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <ros/ros.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <memory>
#include <string>
#include <vector>

namespace py = pybind11;

class FastCollisionChecker {
public:
    FastCollisionChecker(const std::string& group_name = "arm")
        : group_name_(group_name)
    {
        // Initialize ROS if not already initialized
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "fast_collision_checker_node", ros::init_options::AnonymousName);
        }

        ros::NodeHandle nh;

        // Set up PlanningSceneMonitor to sync with MoveIt's planning scene
        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

        if (!planning_scene_monitor_->getPlanningScene()) {
            throw std::runtime_error("Failed to get planning scene.");
        }

        planning_scene_monitor_->startSceneMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();
        planning_scene_monitor_->startStateMonitor();

        planning_scene_ = planning_scene_monitor_->getPlanningScene();

        kmodel_ = planning_scene_->getRobotModel();
        kstate_ = std::make_shared<moveit::core::RobotState>(kmodel_);
        kstate_->setToDefaultValues();

        joint_model_group_ = kmodel_->getJointModelGroup(group_name_);
        if (!joint_model_group_) {
            throw std::runtime_error("Joint model group '" + group_name_ + "' not found.");
        }

        joint_names_ = joint_model_group_->getVariableNames();
    }

    bool isStateValid(const std::vector<double>& joint_values) {
        if (joint_values.size() != joint_names_.size()) {
            throw std::runtime_error("Expected " + std::to_string(joint_names_.size()) +
                                    " joint values, but got " + std::to_string(joint_values.size()));
        }

        // Update the planning scene from the monitor
        planning_scene_monitor_->requestPlanningSceneState();
        planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);

        // Use the updated planning scene
        planning_scene_ = planning_scene_monitor_->getPlanningScene();

        kstate_->setJointGroupPositions(joint_model_group_, joint_values);
        kstate_->update();

        // Perform full collision check (self + world)
        return !planning_scene_->isStateColliding(*kstate_, group_name_, false);
    }

    bool isStateSelfColliding(const std::vector<double>& joint_values) {
        if (joint_values.size() != joint_names_.size()) {
            throw std::runtime_error("Expected " + std::to_string(joint_names_.size()) +
                                     " joint values, but got " + std::to_string(joint_values.size()));
        }

        kstate_->setJointGroupPositions(joint_model_group_, joint_values);
        kstate_->update();

        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        planning_scene_->checkSelfCollision(req, res, *kstate_);

        return res.collision;
    }

    std::vector<std::string> getJointNames() const {
        return joint_names_;
    }

private:
    std::string group_name_;
    moveit::core::RobotModelConstPtr kmodel_;
    planning_scene::PlanningScenePtr planning_scene_;
    moveit::core::RobotStatePtr kstate_;
    const moveit::core::JointModelGroup* joint_model_group_;
    std::vector<std::string> joint_names_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
};

// pybind11 bindings
PYBIND11_MODULE(fast_collision_checker, m) {
    py::class_<FastCollisionChecker>(m, "FastCollisionChecker")
        .def(py::init<const std::string&>(), py::arg("group_name") = "arm")
        .def("is_state_valid", &FastCollisionChecker::isStateValid)
        .def("is_state_self_colliding", &FastCollisionChecker::isStateSelfColliding)
        .def("get_joint_names", &FastCollisionChecker::getJointNames);
}