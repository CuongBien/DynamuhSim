#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <random>
#include <string>
#include <iomanip>
#include <nlohmann/json.hpp>
#include "RVO.h"

using json = nlohmann::json;

struct Point {
    double x, y;
};

double PerpDistanceToLine(double px, double py, double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    double lenSq = dx * dx + dy * dy;
    if (lenSq < 1e-6) return std::hypot(px - x1, py - y1);
    double cross = std::abs((px - x1) * dy - (py - y1) * dx);
    return cross / std::sqrt(lenSq);
}

int main() {
    std::cout << "[Dataset Generator] Simulating 2,500 ground-truth interaction trajectories...\n";

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist_unit(0.0, 1.0);

    const int SAMPLES_PER_CLASS = 500;
    const std::vector<std::string> personas = {"H1", "H2", "H3", "H4", "H5"};
    const std::vector<std::string> scenarios = {"head_on", "crossing", "oblique", "same_direction"};

    json dataset = json::array();

    int ep_id = 0;
    for (const auto &p_name : personas) {
        for (int i = 0; i < SAMPLES_PER_CLASS; ++i) {
            ep_id++;
            std::string sc = scenarios[i % scenarios.size()];

            // Sample scenario parameters
            double robot_speed = 0.4 + 0.3 * dist_unit(rng); // 0.4 to 0.7 m/s
            double human_speed = 0.65 + 0.35 * dist_unit(rng); // 0.65 to 1.0 m/s

            Point r_start{0.0, 0.0};
            Point r_goal{6.0, 0.0};

            Point h_start{0.0, 0.0};
            Point h_goal{0.0, 0.0};

            if (sc == "head_on") {
                double y_off = -0.35 + 0.70 * dist_unit(rng); // offset [-0.35, 0.35]
                double dist_spawn = 4.0 + 2.0 * dist_unit(rng); // [4.0, 6.0]
                h_start = {dist_spawn, y_off};
                h_goal = {-1.5, y_off};
            } else if (sc == "crossing") {
                double x_cross = 2.0 + 1.5 * dist_unit(rng); // [2.0, 3.5]
                bool south_to_north = (dist_unit(rng) > 0.5);
                h_start = {x_cross, south_to_north ? -2.5 : 2.5};
                h_goal = {x_cross, south_to_north ? 2.5 : -2.5};
            } else if (sc == "oblique") {
                double x_spawn = 3.5 + 1.5 * dist_unit(rng);
                bool slant = (dist_unit(rng) > 0.5);
                h_start = {x_spawn, slant ? -1.8 : 1.8};
                h_goal = {x_spawn - 4.0, slant ? 1.8 : -1.8};
            } else if (sc == "same_direction") {
                double y_off = -0.25 + 0.50 * dist_unit(rng);
                h_start = {1.8 + 0.8 * dist_unit(rng), y_off};
                h_goal = {6.5, y_off};
                human_speed = 0.35 + 0.15 * dist_unit(rng); // Slower, robot overtakes
            }

            // Persona specific attributes
            float alpha = 0.50f;
            bool ignores_robot = false;
            double h4_speed = human_speed;

            if (p_name == "H1") {
                alpha = 0.50f;
                ignores_robot = false;
            } else if (p_name == "H2") {
                // Uniformly sample alpha in [0.10, 0.35]
                alpha = 0.10f + 0.25f * static_cast<float>(dist_unit(rng));
                ignores_robot = false;
            } else if (p_name == "H3") {
                alpha = 0.0f;
                ignores_robot = true;
            } else if (p_name == "H4") {
                alpha = 0.0f;
                ignores_robot = false;
                h4_speed = 0.0;
            } else if (p_name == "H5") {
                alpha = 0.0f;
                ignores_robot = true; // In H5, ORCA ignores robot; soft repulsion handles it
            }

            // Simulation parameters: dt = 0.05s, 60 steps = 3.0s (output at 10 Hz = 30 steps)
            const float dt = 0.05f;
            const int total_steps = 60;
            RVO::RVOSimulator sim(dt, 10.0f, 10, 5.0f, 5.0f, 0.30f, 1.2f);

            // Add robot
            RVO::Vector2 r_dir = normalize(RVO::Vector2(r_goal.x - r_start.x, r_goal.y - r_start.y));
            size_t r_idx = sim.addAgent(RVO::Vector2(r_start.x, r_start.y), 10.0f, 10, 5.0f, 5.0f, 0.35f, 1.2f, r_dir * robot_speed);
            sim.setRobotAgentIndex(r_idx);
            sim.setAgentPrefVelocity(r_idx, r_dir * robot_speed);

            // Add human
            RVO::Vector2 h_dir(0.0f, 0.0f);
            if (p_name != "H4") {
                h_dir = normalize(RVO::Vector2(h_goal.x - h_start.x, h_goal.y - h_start.y));
            }
            size_t h_idx = sim.addAgent(RVO::Vector2(h_start.x, h_start.y), 10.0f, 10, 5.0f, 5.0f, 0.28f, 1.2f, h_dir * h4_speed);
            sim.setAgentPrefVelocity(h_idx, h_dir * h4_speed);
            sim.setAgentRobotResponsibility(h_idx, alpha);
            sim.setAgentIgnoresRobot(h_idx, ignores_robot);

            // H5 parameters
            double sf_A = 2.0 + 1.0 * dist_unit(rng); // 2.0 to 3.0
            double sf_B = 0.35 + 0.15 * dist_unit(rng); // 0.35 to 0.50
            double h5_vx = h_dir.x() * human_speed;
            double h5_vy = h_dir.y() * human_speed;
            double tau = 0.25 + 0.10 * dist_unit(rng);

            json traj = json::array();
            double max_lat_dev = 0.0;
            double min_dist_robot = 999.0;

            for (int step = 0; step < total_steps; ++step) {
                // Update robot forward motion
                sim.setAgentPrefVelocity(r_idx, r_dir * robot_speed);

                if (p_name == "H5") {
                    // Soft Social Force computation
                    RVO::Vector2 hpos = sim.getAgentPosition(h_idx);
                    RVO::Vector2 rpos = sim.getAgentPosition(r_idx);
                    double dx = hpos.x() - rpos.x();
                    double dy = hpos.y() - rpos.y();
                    double dist = std::hypot(dx, dy);
                    double dSurf = std::max(0.0, dist - (0.28 + 0.35));

                    double desiredVx = h_dir.x() * human_speed;
                    double desiredVy = h_dir.y() * human_speed;

                    if (dist > 1e-4 && dSurf < 3.0) {
                        double fRep = sf_A * std::exp(-dSurf / sf_B);
                        desiredVx += (dx / dist) * fRep;
                        desiredVy += (dy / dist) * fRep;
                        double dspeed = std::hypot(desiredVx, desiredVy);
                        if (dspeed > 1.2) {
                            desiredVx = (desiredVx / dspeed) * 1.2;
                            desiredVy = (desiredVy / dspeed) * 1.2;
                        }
                    }

                    // LPF
                    double alpha_lpf = dt / tau;
                    h5_vx += alpha_lpf * (desiredVx - h5_vx);
                    h5_vy += alpha_lpf * (desiredVy - h5_vy);
                    sim.setAgentPrefVelocity(h_idx, RVO::Vector2(h5_vx, h5_vy));
                } else if (p_name == "H4") {
                    sim.setAgentPrefVelocity(h_idx, RVO::Vector2(0.0f, 0.0f));
                } else {
                    sim.setAgentPrefVelocity(h_idx, h_dir * human_speed);
                }

                sim.doStep();

                RVO::Vector2 rpos = sim.getAgentPosition(r_idx);
                RVO::Vector2 rvel = sim.getAgentVelocity(r_idx);
                RVO::Vector2 hpos = sim.getAgentPosition(h_idx);
                RVO::Vector2 hvel = sim.getAgentVelocity(h_idx);

                double d_to_r = std::hypot(hpos.x() - rpos.x(), hpos.y() - rpos.y());
                if (d_to_r < min_dist_robot) min_dist_robot = d_to_r;

                double lat_dev = 0.0;
                if (p_name != "H4") {
                    lat_dev = PerpDistanceToLine(hpos.x(), hpos.y(), h_start.x, h_start.y, h_goal.x, h_goal.y);
                } else {
                    lat_dev = std::hypot(hpos.x() - h_start.x, hpos.y() - h_start.y);
                }
                if (lat_dev > max_lat_dev) max_lat_dev = lat_dev;

                // Sample at 10 Hz (every 2 steps)
                if (step % 2 == 1) {
                    json pt;
                    pt["t"] = (step + 1) * dt;
                    pt["rx"] = rpos.x();
                    pt["ry"] = rpos.y();
                    pt["rvx"] = rvel.x();
                    pt["rvy"] = rvel.y();
                    pt["hx"] = hpos.x();
                    pt["hy"] = hpos.y();
                    pt["hvx"] = hvel.x();
                    pt["hvy"] = hvel.y();
                    pt["d_robot"] = d_to_r;
                    pt["lat_dev"] = lat_dev;
                    traj.push_back(pt);
                }
            }

            json episode;
            episode["id"] = ep_id;
            episode["persona"] = p_name;
            episode["scenario"] = sc;
            episode["alpha"] = alpha;
            episode["max_lateral_dev"] = max_lat_dev;
            episode["min_dist_robot"] = min_dist_robot;
            episode["robot_speed"] = robot_speed;
            episode["human_speed"] = (p_name == "H4") ? 0.0 : human_speed;
            episode["trajectory"] = traj;
            dataset.push_back(episode);
        }
    }

    std::ofstream out_file("experiments/pedestrian_persona_benchmark/results/datasets/ground_truth_dataset.json");
    out_file << dataset.dump();
    out_file.close();

    std::cout << "[Dataset Generator] Done! Successfully exported " << dataset.size() 
              << " episodes to experiments/pedestrian_persona_benchmark/results/datasets/ground_truth_dataset.json\n";
    return 0;
}
