#include "orca_manager.hpp"

#define orca_log "[orca]"

namespace RVO 
{
    void orca_manager::update_state(
        Eigen::Vector3f pos, 
        Eigen::Vector3f vel,
        float rad,
        uint id)
    {
        if (myid == id)
        {
            my_state.eval.position_ = pos;
            my_state.eval.velocity_ = vel;
            my_state.eval.radius_ = rad;
            my_state.t = std::chrono::system_clock::now();
            return;
        }

        auto agent = rvo_agents.find(id);
        if (agent == rvo_agents.end())
        {
            orca_agent oa;
            oa.eval.position_ = pos;
            oa.eval.velocity_ = vel;
            oa.eval.radius_ = rad;
            oa.t = std::chrono::system_clock::now();
            rvo_agents.insert({id, oa});
        }
        else
        {
            agent->second.eval.position_ = pos;
            agent->second.eval.velocity_ = vel;
            agent->second.eval.radius_ = rad;
            agent->second.t = std::chrono::system_clock::now();
        }
    }

    bool orca_manager::run(
        Eigen::Vector3f &vel)
    {
        kdtree *kd_tree;
        kd_tree = kd_create(3);
        uint total_processed = 0;
        for (auto &[key, agent] : rvo_agents)
        {
            // check if time is out of the threshold
            std::chrono::duration<double> elapsed = 
                std::chrono::system_clock::now() - agent.t;
            if (elapsed.count() > t_tolerance)
                continue;
            Eval_agent *node = new Eval_agent;
            node->position_ = agent.eval.position_;
            node->velocity_ = agent.eval.velocity_;
            node->radius_ = agent.eval.radius_;
            int v = kd_insert3(
                kd_tree, agent.eval.position_.x(), 
                agent.eval.position_.y(), 
                agent.eval.position_.z(),
                node);
            total_processed++;
        }

        if (total_processed == 0)
            return false;
        
        struct kdres *neighbours;
        neighbours = kd_nearest_range3(
            kd_tree, my_state.eval.position_.x(), 
            my_state.eval.position_.y(), 
            my_state.eval.position_.z(),
            comm_radius);
        
        if (kd_res_size(neighbours) == 0)
            return false;

        // clear agent neighbour before adding in new neighbours and obstacles
        rvo2_agent->clearAgentNeighbor();

        while (!kd_res_end(neighbours))
        {
            double pos[3];
            Eval_agent *agent = 
                (Eval_agent*)kd_res_item(neighbours, pos);
            
            rvo2_agent->insertAgentNeighbor(
                *agent, comm_radius);
            // store range query result so that we dont need to query again for rewire;
            kd_res_next(neighbours); // go to next in kd tree range query result
        }

        kd_free(kd_tree);

        rvo2_agent->updateState(
            my_state.eval.position_, 
            my_state.eval.velocity_, 
            vel);

        rvo2_agent->computeNewVelocity();
        vel = rvo2_agent->getVelocity();
        return true;
    }
}