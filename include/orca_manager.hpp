#ifndef ORCA_MANAGER_H_
#define ORCA_MANAGER_H_

#include "orca.hpp"

#define orca_log "[orca]"

namespace RVO 
{
    class orca_manager
    {
      private:
          struct orca_agent
          {
              RVO::Eval_agent eval;
              std::chrono::system_clock::time_point t;
          };

          std::map<uint, orca_agent> rvo_agents;
          orca_agent my_state;
          uint myid;
          YAML::Node params;
          float planning_rate;
          float max_velocity;
          float protected_zone;
          float buffer_scale;
          float t_tolerance;
          float comm_radius;
          uint max_neighbours;
          std::pair<float, float> height_range;
          std::shared_ptr<RVO::Agent> rvo2_agent;
      
      public:
          orca_manager(uint _myid, const YAML::Node& _params) : 
              myid(_myid), params(_params)
          {
              fmt::print(fg(fmt::color::blue),
                  "{} Initialize orca for (Agent{}) \n",
                  orca_log, myid);
              planning_rate = params["orca"]["rate"].as<float>();
              max_velocity = params["orca"]["max_vel"].as<float>();
              protected_zone = params["orca"]["p_z"].as<float>();
              buffer_scale = params["orca"]["buf"].as<float>();
              max_neighbours = params["orca"]["max_neigh"].as<float>();
              t_tolerance = params["orca"]["t_tolerance"].as<float>();
              comm_radius = params["orca"]["comm_radius"].as<float>();
              height_range = std::pair<float, float>(
                  params["orca"]["height_range"][0].as<float>(),
                  params["orca"]["height_range"][1].as<float>()
              );
          
              rvo2_agent = std::make_shared<RVO::Agent>(
                  RVO::Agent(
                      myid, (1/planning_rate), 
                      max_neighbours, max_velocity, 
                      comm_radius, protected_zone, 
                      buffer_scale * 1/planning_rate,
                      height_range.first, height_range.second));
          };

          void update_state(
              Eigen::Vector3f pos, 
              Eigen::Vector3f vel,
              float rad,
              uint id);
            
          bool run(
              Eigen::Vector3f &vel);
    };
} /* namespace RVO */

#endif /* RVO3D_AGENT_H_ */