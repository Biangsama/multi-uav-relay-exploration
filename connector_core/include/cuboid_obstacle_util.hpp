#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace cuboid {

struct obstacle_t
{
    int id{};
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    double size_x{};
    double size_y{};
    double size_z{};
};

struct triangle_t
{
    Eigen::Vector3d p1;
    Eigen::Vector3d p2;
    Eigen::Vector3d p3;
};

inline std::vector<triangle_t> obstacle_to_triangles_list(obstacle_t obst)
{
    std::vector<triangle_t> triangles;

    const double half_x = obst.size_x / 2.0;
    const double half_y = obst.size_y / 2.0;
    const double half_z = obst.size_z / 2.0;

    Eigen::Vector3d vertices[8] = {
        {obst.center.x() - half_x, obst.center.y() - half_y, obst.center.z() - half_z},
        {obst.center.x() + half_x, obst.center.y() - half_y, obst.center.z() - half_z},
        {obst.center.x() + half_x, obst.center.y() + half_y, obst.center.z() - half_z},
        {obst.center.x() - half_x, obst.center.y() + half_y, obst.center.z() - half_z},
        {obst.center.x() - half_x, obst.center.y() - half_y, obst.center.z() + half_z},
        {obst.center.x() + half_x, obst.center.y() - half_y, obst.center.z() + half_z},
        {obst.center.x() + half_x, obst.center.y() + half_y, obst.center.z() + half_z},
        {obst.center.x() - half_x, obst.center.y() + half_y, obst.center.z() + half_z},
    };

    triangles.push_back({vertices[0], vertices[1], vertices[2]});
    triangles.push_back({vertices[0], vertices[2], vertices[3]});
    triangles.push_back({vertices[4], vertices[6], vertices[5]});
    triangles.push_back({vertices[4], vertices[7], vertices[6]});
    triangles.push_back({vertices[0], vertices[3], vertices[7]});
    triangles.push_back({vertices[0], vertices[7], vertices[4]});
    triangles.push_back({vertices[1], vertices[6], vertices[5]});
    triangles.push_back({vertices[1], vertices[2], vertices[6]});
    triangles.push_back({vertices[3], vertices[2], vertices[6]});
    triangles.push_back({vertices[3], vertices[6], vertices[7]});
    triangles.push_back({vertices[0], vertices[5], vertices[4]});
    triangles.push_back({vertices[0], vertices[1], vertices[5]});
    return triangles;
}

inline Eigen::Vector3d get_nearest_point_from_obstacle(const Eigen::Vector3d& point,
                                                       const obstacle_t& obst)
{
    const double half_x = obst.size_x / 2.0;
    const double half_y = obst.size_y / 2.0;
    const double half_z = obst.size_z / 2.0;

    return Eigen::Vector3d(
        std::clamp(point.x(), obst.center.x() - half_x, obst.center.x() + half_x),
        std::clamp(point.y(), obst.center.y() - half_y, obst.center.y() + half_y),
        std::clamp(point.z(), obst.center.z() - half_z, obst.center.z() + half_z));
}

inline bool segment_cuboid_intersects(const Eigen::Vector3d& p0,
                                      const Eigen::Vector3d& p1,
                                      const obstacle_t& box)
{
    float tmin = 0.0f;
    float tmax = 1.0f;
    const Eigen::Vector3d min(box.center.x() - box.size_x / 2.0,
                              box.center.y() - box.size_y / 2.0,
                              box.center.z() - box.size_z / 2.0);
    const Eigen::Vector3d max(box.center.x() + box.size_x / 2.0,
                              box.center.y() + box.size_y / 2.0,
                              box.center.z() + box.size_z / 2.0);
    const Eigen::Vector3d d = p1 - p0;
    for (int i = 0; i < 3; ++i)
    {
        const float p = (i == 0 ? p0.x() : (i == 1 ? p0.y() : p0.z()));
        const float di = (i == 0 ? d.x() : (i == 1 ? d.y() : d.z()));
        const float bmin = (i == 0 ? min.x() : (i == 1 ? min.y() : min.z()));
        const float bmax = (i == 0 ? max.x() : (i == 1 ? max.y() : max.z()));

        if (std::abs(di) < 1e-6f)
        {
            if (p < bmin || p > bmax) return false;
        }
        else
        {
            float t1 = (bmin - p) / di;
            float t2 = (bmax - p) / di;
            if (t1 > t2) std::swap(t1, t2);
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);
            if (tmin > tmax) return false;
        }
    }
    return true;
}

inline std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>> segment_cuboid_intersection(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    const obstacle_t& obst)
{
    const Eigen::Vector3d min(obst.center.x() - obst.size_x / 2.0,
                              obst.center.y() - obst.size_y / 2.0,
                              obst.center.z() - obst.size_z / 2.0);
    const Eigen::Vector3d max(obst.center.x() + obst.size_x / 2.0,
                              obst.center.y() + obst.size_y / 2.0,
                              obst.center.z() + obst.size_z / 2.0);

    const Eigen::Vector3d direction = p2 - p1;
    Eigen::Vector3d t_min;
    Eigen::Vector3d t_max;

    for (int i = 0; i < 3; ++i)
    {
        if (direction[i] != 0)
        {
            t_min[i] = (min[i] - p1[i]) / direction[i];
            t_max[i] = (max[i] - p1[i]) / direction[i];
            if (t_min[i] > t_max[i]) std::swap(t_min[i], t_max[i]);
        }
        else
        {
            if (p1[i] < min[i] || p1[i] > max[i]) return std::nullopt;
            t_min[i] = -std::numeric_limits<double>::infinity();
            t_max[i] = std::numeric_limits<double>::infinity();
        }
    }

    const double t_enter = std::max({t_min[0], t_min[1], t_min[2]});
    const double t_exit = std::min({t_max[0], t_max[1], t_max[2]});

    if (t_enter > t_exit || t_enter < 0 || t_exit > 1) return std::nullopt;

    return std::make_pair(p1 + t_enter * direction, p1 + t_exit * direction);
}

inline float computeTranslationDistance(const Eigen::Vector3d& u1,
                                        const Eigen::Vector3d& u2,
                                        const obstacle_t& obst,
                                        const Eigen::Vector3d& dir)
{
    float low = 0.0f;
    float high = 1000.0f;
    const int iterations = 20;
    for (int i = 0; i < iterations; ++i)
    {
        const float mid = (low + high) * 0.5f;
        const Eigen::Vector3d offset = dir * mid;
        if (segment_cuboid_intersects(u1 + offset, u2 + offset, obst))
            low = mid;
        else
            high = mid;
    }
    return high;
}

inline std::pair<Eigen::Vector3d, Eigen::Vector3d> computeNonIntersectingLine(
    const Eigen::Vector3d& u1,
    const Eigen::Vector3d& u2,
    const obstacle_t& box)
{
    if (!segment_cuboid_intersects(u1, u2, box)) return {u1, u2};

    const Eigen::Vector3d d = (u1 - u2).normalized();
    Eigen::Vector3d best_offset = Eigen::Vector3d::Zero();
    float min_translation = std::numeric_limits<float>::max();

    const std::vector<Eigen::Vector3d> candidates = {
        Eigen::Vector3d(1, 0, 0),  Eigen::Vector3d(-1, 0, 0), Eigen::Vector3d(0, 1, 0),
        Eigen::Vector3d(0, -1, 0), Eigen::Vector3d(0, 0, 1),  Eigen::Vector3d(0, 0, -1),
    };

    for (const auto& v : candidates)
    {
        Eigen::Vector3d v_proj = v - d * v.dot(d);
        if (v_proj.norm() < 1e-6f) continue;
        v_proj.normalize();

        const float translation = computeTranslationDistance(u1, u2, box, v_proj);
        if (translation < min_translation)
        {
            min_translation = translation;
            best_offset = v_proj * translation;
        }
    }

    return {u1 + best_offset, u2 + best_offset};
}

inline bool point_inside_cuboid(const Eigen::Vector3d& point, const obstacle_t& obst)
{
    const Eigen::Vector3d min(obst.center.x() - obst.size_x / 2.0,
                              obst.center.y() - obst.size_y / 2.0,
                              obst.center.z() - obst.size_z / 2.0);
    const Eigen::Vector3d max(obst.center.x() + obst.size_x / 2.0,
                              obst.center.y() + obst.size_y / 2.0,
                              obst.center.z() + obst.size_z / 2.0);

    return point.x() >= min.x() && point.x() <= max.x() && point.y() >= min.y() &&
           point.y() <= max.y() && point.z() >= min.z() && point.z() <= max.z();
}

inline obstacle_t inflate_obst(const obstacle_t& obst, double inflation)
{
    obstacle_t inflated_obst = obst;
    inflated_obst.size_x += inflation;
    inflated_obst.size_y += inflation;
    inflated_obst.size_z += inflation;
    return inflated_obst;
}

}  // namespace cuboid
