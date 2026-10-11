#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <random>
#include <string>
#include <iomanip>
#include <algorithm>
#include <nlohmann/json.hpp>
#include "RVO.h"

using json = nlohmann::json;

struct Point {
    double x, y;
};

// Compute closest point of approach (CPA) between human and robot under constant velocity
void computeCPA(double rx, double ry, double rvx, double rvy,
                double hx, double hy, double hvx, double hvy,
                double &d_cpa, double &t_cpa) {
    double dx = hx - rx;
    double dy = hy - ry;
    double dvx = hvx - rvx;
    double dvy = hvy - rvy;
    double v_sq = dvx * dvx + dvy * dvy;

    if (v_sq < 1e-6) {
        t_cpa = 0.0;
        d_cpa = std::hypot(dx, dy);
        return;
    }

    t_cpa = -(dx * dvx + dy * dvy) / v_sq;
    if (t_cpa < 0.0) {
        t_cpa = 0.0;
        d_cpa = std::hypot(dx, dy);
    } else {
        double cpa_x = dx + t_cpa * dvx;
        double cpa_y = dy + t_cpa * dvy;
        d_cpa = std::hypot(cpa_x, cpa_y);
    }
}

int main() {
    std::cout << "========================================================================\n";
    std::cout << "[Step 1] Simulating 5,000 High-Fidelity Scenarios for H1-H5 Benchmark\n";
    std::cout << "========================================================================\n";

    const int SAMPLES_PER_CLASS = 1000;
    const std::vector<std::string> personas = {"H1", "H2", "H3", "H4", "H5"};
    const std::vector<std::string> approaches = {"head_on", "crossing", "oblique", "same_direction"};
    const std::vector<std::string> geometries = {"open_6x6", "corridor_09", "corridor_12"};

    json dataset = json::array();
    json metadata_list = json::array();

    int total_scenarios = 0;
    const unsigned int MASTER_SEED = 20261011;

    for (const auto &p_name : personas) {
        std::cout << "  Generating 1,000 scenarios for persona " << p_name << "...\n" << std::flush;
        for (int i = 0; i < SAMPLES_PER_CLASS; ++i) {
            int scenario_id = total_scenarios++;
            unsigned int scenario_seed = MASTER_SEED + scenario_id * 31 + 7;
            std::mt19937 rng(scenario_seed);
            std::uniform_real_distribution<double> dist_unit(0.0, 1.0);

            // Systematic allocation: 250 per approach
            std::string app_type = approaches[(i / 250) % 4];
            // Geometries: cycle open (84), corr_09 (83), corr_12 (83)
            int geo_idx = (i % 3);
            std::string geom = geometries[geo_idx];
            double corr_width = (geom == "corridor_09") ? 0.90 : ((geom == "corridor_12") ? 1.20 : 6.0);
            double y_wall_bound = corr_width / 2.0;

            // Background humans: 0 to 3
            int num_bg = static_cast<int>(dist_unit(rng) * 4); // 0, 1, 2, 3
            if (num_bg > 3) num_bg = 3;

            // Target human preferred speed
            double human_pref_speed = 0.80 + 0.60 * dist_unit(rng); // [0.80, 1.40] m/s

            // Persona specific attributes
            float alpha = 0.50f;
            bool ignores_robot = false;
            double h_actual_speed = human_pref_speed;
            double sfm_A = 0.0, sfm_B = 0.0, sfm_tau = 0.0;

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
                h_actual_speed = 0.0;
            } else if (p_name == "H5") {
                alpha = 0.0f;
                ignores_robot = true; // ORCA ignores; SFM computes soft forces
                sfm_A = 2.0 + 1.5 * dist_unit(rng); // [2.0, 3.5] N
                sfm_B = 0.35 + 0.25 * dist_unit(rng); // [0.35, 0.60] m
                sfm_tau = 0.20 + 0.20 * dist_unit(rng); // [0.20, 0.40] s
            }

            // Robot motion mode: 1 (straight), 2 (slow), 3 (stop_and_go), 4 (turning/weave)
            int robot_mode = 1 + static_cast<int>(dist_unit(rng) * 4);
            if (robot_mode > 4) robot_mode = 4;
            double r_base_speed = (robot_mode == 2) ? (0.15 + 0.10 * dist_unit(rng)) : (0.35 + 0.20 * dist_unit(rng)); // max 0.55 m/s

            // Setup Start & Goal positions
            Point r_start{0.0, 0.0}, r_goal{5.0, 0.0};
            Point h_start{0.0, 0.0}, h_goal{0.0, 0.0};

            double max_y_offset = (geom == "open_6x6") ? 0.8 : (y_wall_bound - 0.25);
            if (max_y_offset < 0.1) max_y_offset = 0.1;

            if (app_type == "head_on") {
                double y_h = -max_y_offset + 2.0 * max_y_offset * dist_unit(rng);
                double dist_spawn = 3.5 + 2.0 * dist_unit(rng); // 3.5 to 5.5 m
                r_start = {0.0, 0.0};
                r_goal = {dist_spawn + 2.0, 0.0};
                h_start = {dist_spawn, y_h};
                h_goal = {-1.5, y_h};
            } else if (app_type == "crossing") {
                if (geom == "open_6x6") {
                    double x_cross = 2.0 + 1.2 * dist_unit(rng);
                    bool south_to_north = (dist_unit(rng) > 0.5);
                    r_start = {0.0, 0.0};
                    r_goal = {5.0, 0.0};
                    h_start = {x_cross, south_to_north ? -2.2 : 2.2};
                    h_goal = {x_cross, south_to_north ? 2.2 : -2.2};
                } else {
                    // Inside corridor: cross diagonally from one wall to the other
                    double x_c_start = 2.0 + 1.5 * dist_unit(rng);
                    double x_c_end = x_c_start + (dist_unit(rng) > 0.5 ? 1.0 : -1.0) * (0.8 + 0.8 * dist_unit(rng));
                    bool south_to_north = (dist_unit(rng) > 0.5);
                    r_start = {0.0, 0.0};
                    r_goal = {5.5, 0.0};
                    h_start = {x_c_start, south_to_north ? (-y_wall_bound + 0.18) : (y_wall_bound - 0.18)};
                    h_goal = {x_c_end, south_to_north ? (y_wall_bound - 0.18) : (-y_wall_bound + 0.18)};
                }
            } else if (app_type == "oblique") {
                double x_spawn = 3.0 + 1.5 * dist_unit(rng);
                bool side = (dist_unit(rng) > 0.5);
                double y_init = side ? -max_y_offset : max_y_offset;
                double y_target = side ? max_y_offset : -max_y_offset;
                r_start = {0.0, 0.0};
                r_goal = {5.5, 0.0};
                h_start = {x_spawn, y_init};
                h_goal = {x_spawn - 3.5, y_target};
            } else if (app_type == "same_direction") {
                double y_off = -max_y_offset * 0.5 + max_y_offset * dist_unit(rng);
                double lead_dist = 1.2 + 0.8 * dist_unit(rng); // human is 1.2 to 2.0 m ahead
                r_start = {0.0, 0.0};
                r_goal = {6.0, 0.0};
                h_start = {lead_dist, y_off};
                h_goal = {6.5, y_off};
                if (p_name != "H4") {
                    h_actual_speed = 0.55 + 0.20 * dist_unit(rng); // human walking slower, robot overtakes
                }
            }

            if (p_name == "H4") {
                h_actual_speed = 0.0;
            }

            // Setup RVO2 Simulator
            // dt = 0.02s (50 Hz), sample every 5 steps (10 Hz = 0.10s)
            const float sim_dt = 0.02f;
            const int total_sim_steps = 300; // 6.0 seconds
            RVO::RVOSimulator sim(sim_dt, 10.0f, 10, 5.0f, 5.0f, 0.30f, 1.5f);

            // Add static walls for corridors
            if (geom == "corridor_09" || geom == "corridor_12") {
                std::vector<RVO::Vector2> bottom_wall = {
                    RVO::Vector2(-6.0f, -y_wall_bound - 0.5f),
                    RVO::Vector2(8.0f, -y_wall_bound - 0.5f),
                    RVO::Vector2(8.0f, -y_wall_bound),
                    RVO::Vector2(-6.0f, -y_wall_bound)
                };
                std::vector<RVO::Vector2> top_wall = {
                    RVO::Vector2(-6.0f, y_wall_bound),
                    RVO::Vector2(8.0f, y_wall_bound),
                    RVO::Vector2(8.0f, y_wall_bound + 0.5f),
                    RVO::Vector2(-6.0f, y_wall_bound + 0.5f)
                };
                sim.addObstacle(bottom_wall);
                sim.addObstacle(top_wall);
                sim.processObstacles();
            }

            // Add Robot agent (agent 0)
            RVO::Vector2 r_dir = normalize(RVO::Vector2(r_goal.x - r_start.x, r_goal.y - r_start.y));
            size_t r_idx = sim.addAgent(RVO::Vector2(r_start.x, r_start.y), 10.0f, 10, 5.0f, 5.0f, 0.35f, 1.2f, r_dir * r_base_speed);
            sim.setRobotAgentIndex(r_idx);
            sim.setAgentPrefVelocity(r_idx, r_dir * r_base_speed);

            // Add Target Human agent (agent 1)
            RVO::Vector2 h_dir(0.0f, 0.0f);
            if (p_name != "H4") {
                double len_h = std::hypot(h_goal.x - h_start.x, h_goal.y - h_start.y);
                if (len_h > 1e-4) {
                    h_dir = RVO::Vector2((h_goal.x - h_start.x) / len_h, (h_goal.y - h_start.y) / len_h);
                }
            }
            size_t h_idx = sim.addAgent(RVO::Vector2(h_start.x, h_start.y), 10.0f, 10, 5.0f, 5.0f, 0.28f, 1.5f, h_dir * h_actual_speed);
            sim.setAgentPrefVelocity(h_idx, h_dir * h_actual_speed);
            sim.setAgentRobotResponsibility(h_idx, alpha);
            sim.setAgentIgnoresRobot(h_idx, ignores_robot);

            // Add Background Humans (agent 2..1+num_bg)
            struct BgHuman {
                size_t id;
                RVO::Vector2 goal;
                double speed;
            };
            std::vector<BgHuman> bg_agents;
            for (int b = 0; b < num_bg; ++b) {
                double bg_x = -1.0 + 5.0 * dist_unit(rng);
                double bg_y = -max_y_offset * 0.8 + 1.6 * max_y_offset * dist_unit(rng);
                // Ensure not right on top of robot or target human
                while (std::hypot(bg_x - r_start.x, bg_y - r_start.y) < 0.8 ||
                       std::hypot(bg_x - h_start.x, bg_y - h_start.y) < 0.8) {
                    bg_x = -1.0 + 5.0 * dist_unit(rng);
                    bg_y = -max_y_offset * 0.8 + 1.6 * max_y_offset * dist_unit(rng);
                }
                double bg_speed = 0.8 + 0.4 * dist_unit(rng);
                double bg_gx = (dist_unit(rng) > 0.5) ? -3.0 : 6.0;
                double bg_gy = -max_y_offset * 0.8 + 1.6 * max_y_offset * dist_unit(rng);
                RVO::Vector2 bg_dir = normalize(RVO::Vector2(bg_gx - bg_x, bg_gy - bg_y));
                size_t b_idx = sim.addAgent(RVO::Vector2(bg_x, bg_y), 8.0f, 8, 4.0f, 4.0f, 0.28f, 1.4f, bg_dir * bg_speed);
                sim.setAgentPrefVelocity(b_idx, bg_dir * bg_speed);
                sim.setAgentRobotResponsibility(b_idx, 0.5f);
                sim.setAgentIgnoresRobot(b_idx, false);
                bg_agents.push_back({b_idx, RVO::Vector2(bg_gx, bg_gy), bg_speed});
            }

            // H5 dynamic state
            double h5_vx = h_dir.x() * h_actual_speed;
            double h5_vy = h_dir.y() * h_actual_speed;

            // Robot current pose (heading theta_r)
            double r_theta = std::atan2(r_dir.y(), r_dir.x());

            json frames = json::array();

            for (int step = 0; step < total_sim_steps; ++step) {
                double cur_t = step * sim_dt;

                // 1. Dynamic robot command based on mode
                double r_target_vx = r_base_speed;
                double r_target_vy = 0.0;
                double r_target_w = 0.0;

                if (robot_mode == 3) {
                    // Stop-and-go
                    if (cur_t >= 1.5 && cur_t <= 3.0) {
                        r_target_vx = 0.0;
                    }
                } else if (robot_mode == 4) {
                    // Lateral turning / weave
                    r_target_w = 0.4 * std::sin(1.8 * cur_t);
                    r_target_vy = 0.15 * std::cos(1.8 * cur_t);
                }
                r_theta += r_target_w * sim_dt;

                // Convert robot body velocities (vx, vy) to world frame
                double cos_th = std::cos(r_theta);
                double sin_th = std::sin(r_theta);
                double r_world_vx = r_target_vx * cos_th - r_target_vy * sin_th;
                double r_world_vy = r_target_vx * sin_th + r_target_vy * cos_th;
                sim.setAgentPrefVelocity(r_idx, RVO::Vector2(r_world_vx, r_world_vy));

                // 2. Persona human logic
                if (p_name == "H5") {
                    RVO::Vector2 hpos = sim.getAgentPosition(h_idx);
                    RVO::Vector2 rpos = sim.getAgentPosition(r_idx);
                    double dx = hpos.x() - rpos.x();
                    double dy = hpos.y() - rpos.y();
                    double dist_r = std::hypot(dx, dy);
                    double dSurf = std::max(0.0, dist_r - (0.28 + 0.35));

                    double fRep_x = 0.0;
                    double fRep_y = 0.0;
                    if (dist_r > 1e-4 && dSurf < 3.0) {
                        double fMag = sfm_A * std::exp(-dSurf / sfm_B);
                        fRep_x = (dx / dist_r) * fMag;
                        fRep_y = (dy / dist_r) * fMag;
                    }

                    // Wall repulsion if corridor
                    if (geom != "open_6x6") {
                        double dist_top = y_wall_bound - hpos.y() - 0.28;
                        double dist_bot = hpos.y() - (-y_wall_bound) - 0.28;
                        if (dist_top < 1.0) {
                            fRep_y -= 2.0 * std::exp(-std::max(0.0, dist_top) / 0.30);
                        }
                        if (dist_bot < 1.0) {
                            fRep_y += 2.0 * std::exp(-std::max(0.0, dist_bot) / 0.30);
                        }
                    }

                    // Background human repulsion
                    for (const auto &bg : bg_agents) {
                        RVO::Vector2 bgpos = sim.getAgentPosition(bg.id);
                        double bg_dx = hpos.x() - bgpos.x();
                        double bg_dy = hpos.y() - bgpos.y();
                        double bg_dist = std::hypot(bg_dx, bg_dy);
                        double bg_dSurf = std::max(0.0, bg_dist - (0.28 + 0.28));
                        if (bg_dist > 1e-4 && bg_dSurf < 1.5) {
                            double f_bg = 1.5 * std::exp(-bg_dSurf / 0.35);
                            fRep_x += (bg_dx / bg_dist) * f_bg;
                            fRep_y += (bg_dy / bg_dist) * f_bg;
                        }
                    }

                    double desiredVx = h_dir.x() * h_actual_speed + fRep_x;
                    double desiredVy = h_dir.y() * h_actual_speed + fRep_y;
                    double des_spd = std::hypot(desiredVx, desiredVy);
                    double max_spd = human_pref_speed * 1.4;
                    if (des_spd > max_spd) {
                        desiredVx = (desiredVx / des_spd) * max_spd;
                        desiredVy = (desiredVy / des_spd) * max_spd;
                    }

                    double alpha_lpf = sim_dt / sfm_tau;
                    h5_vx += alpha_lpf * (desiredVx - h5_vx);
                    h5_vy += alpha_lpf * (desiredVy - h5_vy);
                    sim.setAgentPrefVelocity(h_idx, RVO::Vector2(h5_vx, h5_vy));
                } else if (p_name == "H4") {
                    sim.setAgentPrefVelocity(h_idx, RVO::Vector2(0.0f, 0.0f));
                } else {
                    sim.setAgentPrefVelocity(h_idx, h_dir * h_actual_speed);
                }

                // 3. Update background humans
                for (const auto &bg : bg_agents) {
                    RVO::Vector2 bpos = sim.getAgentPosition(bg.id);
                    RVO::Vector2 b_to_g = bg.goal - bpos;
                    double d_bg = abs(b_to_g);
                    if (d_bg > 1e-3) {
                        sim.setAgentPrefVelocity(bg.id, (b_to_g / d_bg) * bg.speed);
                    } else {
                        sim.setAgentPrefVelocity(bg.id, RVO::Vector2(0.0f, 0.0f));
                    }
                }

                sim.doStep();

                // Sample every 5 steps (0.10s = 10 Hz)
                if (step % 5 == 4) {
                    RVO::Vector2 cur_rpos = sim.getAgentPosition(r_idx);
                    RVO::Vector2 cur_rvel = sim.getAgentVelocity(r_idx);
                    RVO::Vector2 cur_hpos = sim.getAgentPosition(h_idx);
                    RVO::Vector2 cur_hvel = sim.getAgentVelocity(h_idx);

                    // Compute nominal CPA at this frame
                    double d_cpa = 0.0, t_cpa = 0.0;
                    computeCPA(cur_rpos.x(), cur_rpos.y(), cur_rvel.x(), cur_rvel.y(),
                               cur_hpos.x(), cur_hpos.y(), cur_hvel.x(), cur_hvel.y(),
                               d_cpa, t_cpa);

                    // Position and velocity in robot body frame:
                    // x_rel: forward along robot heading, y_rel: left
                    double dx_world = cur_hpos.x() - cur_rpos.x();
                    double dy_world = cur_hpos.y() - cur_rpos.y();
                    double c_th = std::cos(r_theta);
                    double s_th = std::sin(r_theta);
                    double x_rel = c_th * dx_world + s_th * dy_world;
                    double y_rel = -s_th * dx_world + c_th * dy_world;

                    // Relative velocity in robot frame
                    double dvx_world = cur_hvel.x() - cur_rvel.x();
                    double dvy_world = cur_hvel.y() - cur_rvel.y();
                    double vx_rel = c_th * dvx_world + s_th * dvy_world;
                    double vy_rel = -s_th * dvx_world + c_th * dvy_world;

                    double d_dist = std::hypot(x_rel, y_rel);
                    double d_angle = std::atan2(y_rel, x_rel);

                    json f;
                    f["t"] = (step + 1) * sim_dt;
                    f["rx"] = cur_rpos.x();
                    f["ry"] = cur_rpos.y();
                    f["r_theta"] = r_theta;
                    f["r_vx"] = r_target_vx;
                    f["r_vy"] = r_target_vy;
                    f["r_w"] = r_target_w;
                    f["hx"] = cur_hpos.x();
                    f["hy"] = cur_hpos.y();
                    f["hvx"] = cur_hvel.x();
                    f["hvy"] = cur_hvel.y();
                    f["x_rel"] = x_rel;
                    f["y_rel"] = y_rel;
                    f["vx_rel"] = vx_rel;
                    f["vy_rel"] = vy_rel;
                    f["dist"] = d_dist;
                    f["angle"] = d_angle;
                    f["d_cpa"] = d_cpa;
                    f["t_cpa"] = t_cpa;

                    frames.push_back(f);
                }
            }

            // Split: 60% Train, 20% Val, 20% Test (modulo 10 gives exact stratification)
            int split_mod = scenario_id % 10;
            std::string split_name = (split_mod < 6) ? "train" : ((split_mod < 8) ? "val" : "test");

            json ep;
            ep["scenario_id"] = scenario_id;
            ep["seed"] = scenario_seed;
            ep["split"] = split_name;
            ep["persona"] = p_name;
            ep["persona_idx"] = (p_name == "H1") ? 0 : ((p_name == "H2") ? 1 : ((p_name == "H3") ? 2 : ((p_name == "H4") ? 3 : 4)));
            ep["geometry"] = geom;
            ep["approach_type"] = app_type;
            ep["robot_mode"] = robot_mode;
            ep["num_background"] = num_bg;
            ep["human_pref_speed"] = human_pref_speed;
            ep["alpha"] = alpha;
            ep["sfm_A"] = sfm_A;
            ep["sfm_B"] = sfm_B;
            ep["sfm_tau"] = sfm_tau;
            ep["frames"] = frames;

            dataset.push_back(ep);

            json meta;
            meta["scenario_id"] = scenario_id;
            meta["seed"] = scenario_seed;
            meta["split"] = split_name;
            meta["persona"] = p_name;
            meta["geometry"] = geom;
            meta["approach_type"] = app_type;
            meta["alpha"] = alpha;
            meta["num_background"] = num_bg;
            metadata_list.push_back(meta);
        }
    }

    std::cout << "[Step 1] Exporting 5,000 scenarios to JSON files...\n" << std::flush;
    std::string out_data_path = "experiments/pedestrian_persona_benchmark/h1_h5_scenarios_dataset.json";
    std::ofstream out_file(out_data_path);
    out_file << dataset.dump();
    out_file.close();

    std::string out_meta_path = "experiments/pedestrian_persona_benchmark/scenarios_metadata.json";
    std::ofstream meta_file(out_meta_path);
    meta_file << metadata_list.dump(2);
    meta_file.close();

    std::cout << "========================================================================\n";
    std::cout << "[Step 1 Complete] Exported " << dataset.size() << " scenarios successfully!\n";
    std::cout << "  - Dataset: " << out_data_path << "\n";
    std::cout << "  - Metadata & Seeds: " << out_meta_path << "\n";
    std::cout << "========================================================================\n";

    return 0;
}
