#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#define DOCTEST_CONFIG_NO_POSIX_SIGNALS
#include "doctest/doctest.h"

#include "orca_manager.hpp"
#include <thread>

namespace ocas
{
    double d2r = 0.0174533;
    double r2d = 57.2958;

    double constrain_angle(double x)
    {
        x = std::fmod(x + M_PI, M_PI * 2);
        if (x < 0)
            x += M_PI * 2;
        return x - M_PI;
    }

    Eigen::Vector3d point_mass_dynamics(
        double max_acc,
        double period,
        Eigen::Vector3d desired_velocity,
        Eigen::Vector3d current_velocity)
    {
        auto total_acc = std::min(
            (desired_velocity - current_velocity).norm() / period, max_acc);
        
        auto acc_vector = 
            (desired_velocity - current_velocity).normalized();
        return acc_vector * period * total_acc + current_velocity;
    }

    TEST_CASE("Simulation Simple Same Axis Crossing")
    {
        std::string test_name = "simple_crossing";

        const auto config_yaml = YAML::LoadFile(
            "config/config.yaml");

        auto time_step = 0.25F;
        auto agents_count = 5;
        auto radius = 30.0F;
        auto height = 10.0F;
        auto protected_zone = 
            config_yaml["orca"]["p_z"].as<float>();
        auto max_neighbours = 
            config_yaml["orca"]["max_neigh"].as<uint>();
        auto max_velocity = 
            config_yaml["orca"]["max_vel"].as<float>();
        auto comm_radius = 
            config_yaml["orca"]["comm_radius"].as<float>();

        /******************************
         * Agent states
         ******************************/
        std::map<int, std::pair<RVO::Eval_agent, RVO::Eval_agent>> agents_states;
        std::map<int, Eigen::Vector3f> agents_commands;

        std::map<int, std::shared_ptr<RVO::orca_manager>> agents_orca;   
        
        for (std::size_t i = 0; i < agents_count; ++i) 
        {
            Eigen::Vector2f xy = radius *
                Eigen::Vector2f(
                    std::cos(1/(float)agents_count * static_cast<float>(i) * 2*M_PI),
                    std::sin(1/(float)agents_count * static_cast<float>(i) * 2*M_PI));
            RVO::Eval_agent eval, dest;
            eval.position_ = Eigen::Vector3f(xy.x(), xy.y(), height);
            dest.position_ = Eigen::Vector3f(-xy.x(), -xy.y(), height);
            eval.velocity_ = dest.velocity_ = 
                (dest.position_ - eval.position_).normalized() * max_velocity/1.5;
            eval.radius_ = dest.radius_ = protected_zone;
            std::pair<RVO::Eval_agent, RVO::Eval_agent> state_cmd(eval, dest);
            agents_states.insert({i, state_cmd});
            agents_commands.insert({i, Eigen::Vector3f::Zero()});

            fmt::print(fg(fmt::color::green),
                "TESTORCA Agent{} pos ({} {} {}) vel ({} {} {})! \n",
                i,
                std::round(eval.position_.x()*100) / 100,
                std::round(eval.position_.y()*100) / 100,
                std::round(eval.position_.z()*100) / 100,
                std::round(eval.velocity_.x()*100) / 100,
                std::round(eval.velocity_.y()*100) / 100,
                std::round(eval.velocity_.z()*100) / 100);

            auto om = std::make_shared<RVO::orca_manager>(
                i, config_yaml);

            agents_orca.emplace(i, om);
        }

        bool completed = false;
        auto iteration = 0;
        while (!completed)
        {
            fmt::print(fg(fmt::color::sky_blue),
                "------------------------------- iteration {} {}s\n",
                iteration, std::round(time_step * iteration * 100) / 100);
            for (auto &orca : agents_orca)
            {
                auto ac =
                    agents_commands.find(orca.first)->second;

                // calculate next odometry reading using ackermann controller
                if (iteration != 0)
                {
                    // check command and then progress the state
                    // point_mass_dynamics(
                    //     ac, state.second, next,
                    //     interval, turnrate);
                    // state.second = next;
                }

                for (auto &other_state : agents_states)
                {
                    if (orca.first == other_state.first)
                        continue;
                    orca.second->update_state(
                        other_state.second.first.position_,
                        other_state.second.first.velocity_,
                        other_state.second.first.radius_,
                        other_state.first
                    );
                }
            }

            /******************************
             * Run test
             ******************************/
            for (auto &orca : agents_orca)
            {
                auto agent = agents_states.find(orca.first);
                auto command = agents_commands.find(orca.first);
                Eigen::Vector3f vel = 
                    (max_velocity/2) * (agent->second.second.position_ - agent->second.first.position_).normalized();
                if (orca.second->run(vel))
                {
                    command->second = vel;
                    fmt::print(fg(fmt::color::sky_blue),
                        "TESTORCA Agent{} orca success! \n", orca.first); 
                }

                fmt::print(fg(fmt::color::sky_blue),
                    "TESTORCA Agent{} command ({} {} {})! \n",
                    orca.first,
                    std::round(command->second.x()*100) / 100,
                    std::round(command->second.y()*100) / 100,
                    std::round(command->second.z()*100) / 100);
            }

            fmt::print(fg(fmt::color::sky_blue), "------------------------------- end\n");
            iteration++;

            std::this_thread::sleep_for(
                std::chrono::milliseconds((int)std::round(time_step * 1000)));
            break;
        }
    } //TEST_CASE
}