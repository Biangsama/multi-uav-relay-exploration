#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <vector>

struct target_t
{
    uint32_t id{};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    float radius{};
    bool global{};
    std::vector<uint32_t> assigned_agents;
    bool is_sink{};
};
