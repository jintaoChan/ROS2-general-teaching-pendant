#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp/duration.hpp>

class TrajectoryGenerator {
public:
    static trajectory_msgs::msg::JointTrajectory generate(
        const std::vector<std::string>& joint_names,
        const std::vector<double>& start,
        const std::vector<double>& goal,
        const std::vector<double>& velocity_limits,
        double velo_ratio,
        double waypoint_dt = 0.05)
    {
        const size_t n = joint_names.size();
        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = joint_names;

        std::vector<double> abs_disp(n);
        std::vector<double> v_max(n);
        std::vector<double> dir(n);

        for (size_t i = 0; i < n; ++i) {
            double d = goal[i] - start[i];
            abs_disp[i] = std::abs(d);
            v_max[i] = std::max(velo_ratio * velocity_limits[i], 1e-9);
            dir[i] = (d >= 0.0) ? 1.0 : -1.0;
        }

        // Acceleration: reach max velocity in accel_time_constant seconds
        constexpr double accel_time_constant = 0.5;

        // Compute per-joint trapezoidal time and take maximum
        double T = 0.0;
        for (size_t i = 0; i < n; ++i) {
            if (abs_disp[i] < 1e-9) continue;
            double a = v_max[i] / accel_time_constant;
            double d_threshold = v_max[i] * v_max[i] / a;
            double t_i;
            if (abs_disp[i] < d_threshold) {
                t_i = 2.0 * std::sqrt(abs_disp[i] / a);
            } else {
                double t_accel = v_max[i] / a;
                t_i = 2.0 * t_accel + (abs_disp[i] - d_threshold) / v_max[i];
            }
            T = std::max(T, t_i);
        }

        T = std::max(T, 0.1);

        // Synchronized profile parameters per joint
        struct Profile {
            double v{0.0};
            double a{0.0};
            double t_accel{0.0};
        };
        std::vector<Profile> profiles(n);

        for (size_t i = 0; i < n; ++i) {
            if (abs_disp[i] < 1e-9) continue;
            double v_tri = 2.0 * abs_disp[i] / T;
            if (v_tri <= v_max[i]) {
                profiles[i].v = v_tri;
                profiles[i].t_accel = T / 2.0;
            } else {
                profiles[i].v = v_max[i];
                profiles[i].t_accel = T - abs_disp[i] / v_max[i];
                if (profiles[i].t_accel < 1e-9) profiles[i].t_accel = 1e-9;
            }
            profiles[i].a = profiles[i].v / profiles[i].t_accel;
        }

        auto makePoint = [&](double t) {
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions.resize(n);
            pt.velocities.resize(n);
            for (size_t i = 0; i < n; ++i) {
                if (abs_disp[i] < 1e-9) {
                    pt.positions[i] = start[i];
                    pt.velocities[i] = 0.0;
                    continue;
                }
                const auto& p = profiles[i];
                double remaining = T - t;
                if (t <= p.t_accel) {
                    pt.positions[i] = start[i] + dir[i] * 0.5 * p.a * t * t;
                    pt.velocities[i] = dir[i] * p.a * t;
                } else if (remaining > p.t_accel) {
                    double accel_dist = 0.5 * p.a * p.t_accel * p.t_accel;
                    pt.positions[i] = start[i] + dir[i] * (accel_dist + p.v * (t - p.t_accel));
                    pt.velocities[i] = dir[i] * p.v;
                } else {
                    pt.positions[i] = goal[i] - dir[i] * 0.5 * p.a * remaining * remaining;
                    pt.velocities[i] = dir[i] * p.a * remaining;
                }
            }
            pt.time_from_start = rclcpp::Duration::from_nanoseconds(
                static_cast<int64_t>(t * 1e9));
            return pt;
        };

        int num_points = std::max(2, static_cast<int>(std::ceil(T / waypoint_dt)) + 1);
        double actual_dt = T / (num_points - 1);
        for (int k = 0; k < num_points; ++k) {
            double t = (k == num_points - 1) ? T : k * actual_dt;
            traj.points.push_back(makePoint(t));
        }

        // Ensure exact goal positions and zero velocity at end
        auto& last = traj.points.back();
        for (size_t i = 0; i < n; ++i) {
            last.positions[i] = goal[i];
            last.velocities[i] = 0.0;
        }

        return traj;
    }
};
