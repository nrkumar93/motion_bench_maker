/* Author: Ramkumar Natarajan */

// ROS
#include <moveit_msgs/MoveItErrorCodes.h>
#include <ros/ros.h>

// Robowflex dataset
#include <motion_bench_maker/setup.h>
#include <motion_bench_maker/parser.h>
#include <motion_bench_maker/utils.h>

// Robowflex library
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/io.h>

// Robowflex search
#include <robowflex_search/search_interface.h>
#include <robowflex_ompl/ompl_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <robowflex_library/trajectory.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    // Enables multiple instances running with the same name
    ros::init(argc, argv, "benchmark_search");
    ros::NodeHandle node("~");

    std::string dataset, results, exp_name;
    std::vector<std::string> planners;
    double time_limit, runs;
    bool sensed, train;

    int start, end;

    std::string exec_name = "benchmark_search";

    // Load ROS params
    size_t error = 0;
    // Parsing the parametes from the param server
    error += !parser::get(exec_name, node, "dataset", dataset);
    error += !parser::get(exec_name, node, "results", results);
    error += !parser::get(exec_name, node, "exp_name", exp_name);
    error += !parser::get(exec_name, node, "time_limit", time_limit);
    error += !parser::get(exec_name, node, "planners", planners);
    error += !parser::get(exec_name, node, "start", start);
    error += !parser::get(exec_name, node, "end", end);
    error += !parser::get(exec_name, node, "runs", runs);
    error += !parser::get(exec_name, node, "sensed", sensed);
    error += !parser::get(exec_name, node, "train", train);

    parser::shutdownIfError(exec_name, error);

    auto setup = std::make_shared<Setup>(dataset);
    auto robot = setup->getRobot();

    // Benchmarker
    Profiler::Options options;
    options.metrics = Profiler::WAYPOINTS | Profiler::CORRECT | Profiler::LENGTH | Profiler::SMOOTHNESS;

    Experiment experiment(exp_name, options, time_limit, runs);
    auto settings = search::Settings();
    settings.interpolate_solutions = false;

    auto planner = setup->createSearchPlanner("", setup->getGroup(), settings);

    int numSamples = (end > 1) ? end : setup->getNumSamples();
    for (int i = start; i <= numSamples; i++)
    {
        const auto &scene = std::make_shared<Scene>(robot);
        if (sensed) {
            // setup->loadSensedScene(i, scene);
            ROS_ERROR("Not supporting sensed scenes now. Only geometric scenes!"); // Mainly because of the line "scene->getCurrentState() = *request->getStartConfiguration();" below
            return -1;
        } else {
            setup->loadGeometricScene(i, scene);
        }

        for (const auto &planner_name : planners)
        {
            const auto &request = std::make_shared<MotionRequestBuilder>(planner, setup->getGroup());
            setup->loadRequest(i, request);

            if (!sensed) {
                scene->getCurrentState() = *request->getStartConfiguration();
            }

            request->setAllowedPlanningTime(time_limit);
            request->setNumPlanningAttempts(1);

            // Set the name of the planner
            if (!request->setConfig(planner_name))
                ROS_ERROR("Did not find planner %s", planner_name.c_str());

            experiment.addQuery(planner_name, scene, planner, request);
        }
    }

    auto results_data = experiment.benchmark(1);

    // For path simplification
    double max_simp_time = 1.0;
    int k=0;
    for (const auto& rd : results_data->data) {
        for (const auto& data_unit : rd.second) {
            ++k;
            ROS_INFO_STREAM("Simplifying path #: " << k << std::endl);
            if (not data_unit->success) {
                ROS_INFO_STREAM("Path plan #: " << k << " was a failure" << std::endl);
                continue;
            }

            auto& mpres = data_unit->response;
            auto& mpscene = data_unit->query.scene;

            ompl::base::SpaceInformationPtr si = convertMoveitPlanningSceneToOMPLSpaceInformation(mpscene->getSceneConst(), setup->getGroup());
            si->setup();
            ompl::geometric::PathSimplifier psimper(si);

            auto ompl_gp = convertMoveItMotionPlanResponseToOMPLPathGeometric(mpres, si);
            try {
                psimper.simplify(ompl_gp, max_simp_time);
            }
            catch (std::exception& e) {
                ROS_INFO_STREAM(e.what() << std::endl);
                ROS_INFO_STREAM("Simplification for path #: " << k << " failed" << std::endl);
                data_unit->simplified_path =  nullptr;

                data_unit->metrics["simplified_l1_length"] = -1;
                data_unit->metrics["simplified_l2_length"] = -1;
                continue;
            } catch (...) {
                ROS_INFO_STREAM("Simplification for path #: " << k << " segfaulted" << std::endl);
                data_unit->simplified_path =  nullptr;

                data_unit->metrics["simplified_l1_length"] = -1;
                data_unit->metrics["simplified_l2_length"] = -1;
                continue;
            }
            auto simp_traj = convertOMPLPathGeometricToMoveItRobotTrajectory(ompl_gp, robot->getModelConst(), setup->getGroup());
            data_unit->simplified_path =  std::make_shared<robowflex::Trajectory>(simp_traj);
            std::cout << "Original path had: " << data_unit->trajectory->getNumWaypoints() << std::endl;
            std::cout << "Original path length: " << data_unit->trajectory->getLength() << std::endl;
            std::cout << "Simplified path had: " << data_unit->simplified_path->getNumWaypoints() << std::endl;
            std::cout << "Simplified path length: " << data_unit->simplified_path->getLength() << std::endl;

            data_unit->metrics["simplified_l1_length"] = robot_trajectory::path_length(*data_unit->simplified_path->getTrajectory());
            data_unit->metrics["simplified_l2_length"] = data_unit->simplified_path->getLength();
        }
    }

    SearchPlanDataSetOutputter output(IO::resolveParent(results));
    std::cout << "Results dir: " << IO::resolveParent(results) << std::endl;
    output.dump(*results_data);

    return 0;
}
