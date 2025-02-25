//
// Created by Ramkumar Natarajan on 2/25/25.
//

#ifndef ROBOWFLEX_DATASET_UTILS_H
#define ROBOWFLEX_DATASET_UTILS_H

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <robowflex_library/trajectory.h>

class MoveItStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
    MoveItStateValidityChecker(const ompl::base::SpaceInformationPtr &si,
                               const planning_scene::PlanningSceneConstPtr &planning_scene,
                               const std::string &group_name)
        : ompl::base::StateValidityChecker(si),
          planning_scene_(planning_scene),
          group_name_(group_name)
    {
    }

    bool isValid(const ompl::base::State *state) const override
    {
        moveit::core::RobotState robot_state(planning_scene_->getRobotModel());
        const moveit::core::JointModelGroup *jmg = robot_state.getJointModelGroup(group_name_);

        std::vector<double> joint_values(jmg->getVariableCount());
        for (size_t i = 0; i < joint_values.size(); ++i)
        {
            joint_values[i] = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
        }

        robot_state.setJointGroupPositions(jmg, joint_values);
        robot_state.update(); // Ensure forward kinematics is updated

        return !planning_scene_->isStateColliding(robot_state, group_name_, true);
    }

private:
    planning_scene::PlanningSceneConstPtr planning_scene_;
    std::string group_name_;
};

ompl::base::SpaceInformationPtr convertMoveitPlanningSceneToOMPLSpaceInformation(
    const planning_scene::PlanningSceneConstPtr &planning_scene,
    const std::string &group_name)
{
    // Get Robot Model and Joint Model Group
    moveit::core::RobotModelConstPtr robot_model = planning_scene->getRobotModel();
    const moveit::core::JointModelGroup *jmg = robot_model->getJointModelGroup(group_name);

    if (!jmg)
    {
        throw std::runtime_error("Invalid Joint Model Group: " + group_name);
    }

    // Create an OMPL State Space
    auto state_space = std::make_shared<ompl::base::RealVectorStateSpace>(jmg->getVariableCount());

    // Set bounds from MoveIt's joint limits
    ompl::base::RealVectorBounds bounds(jmg->getVariableCount());
    const std::vector<const moveit::core::JointModel *> &joint_models = jmg->getActiveJointModels();

    for (size_t i = 0; i < joint_models.size(); ++i)
    {
        const moveit::core::VariableBounds &joint_bounds = joint_models[i]->getVariableBounds(joint_models[i]->getName());
        bounds.setLow(i, joint_bounds.min_position_);
        bounds.setHigh(i, joint_bounds.max_position_);
    }
    state_space->setBounds(bounds);

    // Create SpaceInformation
    auto si = std::make_shared<ompl::base::SpaceInformation>(state_space);

    // Set a validity checker that uses MoveIt's collision detection
    si->setStateValidityChecker(std::make_shared<MoveItStateValidityChecker>(si, planning_scene, group_name));

    si->setup();
    return si;
}

ompl::geometric::PathGeometric convertMoveItMotionPlanResponseToOMPLPathGeometric(
    const planning_interface::MotionPlanResponse& response,
    const ompl::base::SpaceInformationPtr& si)
{
    ompl::geometric::PathGeometric ompl_path(si);

    // Ensure the response has a valid trajectory
    if (!response.trajectory_ || response.trajectory_->getWayPointCount() == 0)
    {
        return ompl_path;
    }

    // Convert each waypoint to an OMPL state
    for (size_t i = 0; i < response.trajectory_->getWayPointCount(); ++i)
    {
        const moveit::core::RobotState& waypoint = response.trajectory_->getWayPoint(i);
        ompl::base::State* state = si->allocState();

        // Copy state values from MoveIt RobotState to OMPL state
        const moveit::core::JointModelGroup* jmg = waypoint.getJointModelGroup(response.trajectory_->getGroupName());
        std::vector<double> joint_values;
        waypoint.copyJointGroupPositions(jmg, joint_values);

        for (size_t j = 0; j < joint_values.size(); ++j)
        {
            state->as<ompl::base::RealVectorStateSpace::StateType>()->values[j] = joint_values[j];
        }

        ompl_path.append(state);
    }

    return ompl_path;
}

robot_trajectory::RobotTrajectory convertOMPLPathGeometricToMoveItRobotTrajectory(
    const ompl::geometric::PathGeometric& ompl_path,
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::string& group_name)
{
    robot_trajectory::RobotTrajectory moveit_trajectory(robot_model, group_name);

    // Ensure we have a valid joint model group
    const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group_name);
    if (!jmg)
    {
        throw std::runtime_error("Invalid Joint Model Group: " + group_name);
    }

    // Create an empty RobotState
    moveit::core::RobotState robot_state(robot_model);
    robot_state.setToDefaultValues();  // Initialize with default values

    for (size_t i = 0; i < ompl_path.getStateCount(); ++i)
    {
        const ompl::base::State* ompl_state = ompl_path.getState(i);

        std::vector<double> joint_values(jmg->getVariableCount());
        for (size_t j = 0; j < joint_values.size(); ++j)
        {
            joint_values[j] = ompl_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[j];
        }

        // Set joint values into the RobotState
        robot_state.setJointGroupPositions(jmg, joint_values);
        moveit_trajectory.addSuffixWayPoint(robot_state, 0.0);  // 0.0 as placeholder for timing
    }

    return moveit_trajectory;
}



#endif //ROBOWFLEX_DATASET_UTILS_H
