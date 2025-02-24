/* Author: Ramkumar Natarajan */

// ROS
#include <moveit_msgs/MoveItErrorCodes.h>
#include <ros/ros.h>

// Robowflex dataset
#include <motion_bench_maker/setup.h>
#include <motion_bench_maker/parser.h>

// Robowflex library
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>

// Robowflex search
#include <robowflex_search/search_interface.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <robowflex_library/log.h>

using namespace robowflex;
int main(int argc, char **argv)
{
    // Enables multiple instances running with the same name
    ros::init(argc, argv, "visualize");
    ros::NodeHandle node("~");

    std::string dataset, config, planner_name;
    bool geometric, sensed, solve, pcd;
    double time_limit;

    std::string exec_name = "visualize";

    // Load ROS params
    size_t error = 0;
    // Parsing the parametes from the param server
    error += !parser::get(exec_name, node, "dataset", dataset);
    error += !parser::get(exec_name, node, "geometric", geometric);
    error += !parser::get(exec_name, node, "sensed", sensed);
    error += !parser::get(exec_name, node, "solve", solve);
    error += !parser::get(exec_name, node, "pcd", pcd);
    error += !parser::get(exec_name, node, "planner_name", planner_name);
    error += !parser::get(exec_name, node, "time_limit", time_limit);
    parser::shutdownIfError(exec_name, error);

    auto setup = std::make_shared<Setup>(dataset);
    auto robot = setup->getRobot();
    auto settings = search::Settings();
    settings.interpolate_solutions = true;

    auto planner = setup->createSearchPlanner("", setup->getGroup(), settings);

    auto rviz = std::make_shared<IO::RVIZHelper>(robot);

    int offset = 1;
    for (int i = offset; i <= setup->getNumSamples(); i++)
    {
        auto scene_geom = std::make_shared<Scene>(robot);
        auto scene_sensed = std::make_shared<Scene>(robot);
        auto scene_pcd = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        if (geometric)
        {
            setup->loadGeometricScene(i, scene_geom);
            rviz->updateScene(scene_geom);
        }
        if (sensed)
        {
            setup->loadSensedScene(i, scene_sensed);
            rviz->updateScene(scene_sensed);
        }
        if (pcd)
        {
            setup->loadPCDScene(i, scene_pcd);
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*scene_pcd, msg);
            msg.header.frame_id = "map";
            rviz->updatePCD(msg);
        }

        auto request = std::make_shared<MotionRequestBuilder>(planner, setup->getGroup());
        setup->loadRequest(i, request);

        // Set the name of the planner
        if (!request->setConfig(planner_name))
            ROS_ERROR("Did not find planner %s", planner_name.c_str());

        auto start = request->getStartConfiguration();
        rviz->visualizeState(start);

        std::vector<double> start_joint_values;
        start->copyJointGroupPositions(setup->getGroup(), start_joint_values);
        std::cout << "start: ";
        for (auto sj : start_joint_values) {
            std::cout << sj << ", ";
        }
        std::cout << std::endl;

        parser::waitForUser("Displaying start:" + std::to_string(i));
        if (geometric)  // Trick to visualize start/goal together
        {
            scene_geom->getCurrentState() = *start;
            rviz->updateScene(scene_geom);
        }

        auto goal = request->getGoalConfiguration();
        rviz->visualizeState(goal);

        std::vector<double> goal_joint_values;
        goal->copyJointGroupPositions(setup->getGroup(), goal_joint_values);
        std::cout << "goal: ";
        for (auto& gj : goal_joint_values) {
            std::cout << gj << ", ";
        }

        parser::waitForUser("Displaying Goal:" + std::to_string(i));

        auto trajectory = std::make_shared<Trajectory>(robot, setup->getGroup());
        if (solve)
        {
            request->setAllowedPlanningTime(time_limit);
            request->setNumPlanningAttempts(1);
            const auto &res = planner->plan(scene_geom, request->getRequestConst());
            if (res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
                trajectory->getTrajectory() = res.trajectory_;
            else
                continue;
        }

        else
            setup->loadTrajectory(i, *start, trajectory);

        trajectory->interpolate(100);

        rviz->updateTrajectory(trajectory->getTrajectoryConst());

        parser::waitForUser("Displaying computed/loaded trajectory:" + std::to_string(i));
    }
    return 0;
}
