#include "custom_corridor_plugins/scenario_presets.hpp"
#include <algorithm>
#include <cmath>
#include <nlohmann/json.hpp>

namespace custom_corridor
{

using json = nlohmann::json;

std::string GetHospitalEasyPresetJson()
{
  return R"({
          "episode_id": 1,
          "humans": [
            {"id": 1, "name": "hospital_walking_human", "mode": "autonomous", "speed": 0.75, "scale": 1.00, "visual_z": 1.00, "proxy_z": 0.85, "waypoints": [[-14.0, 0.0], [-2.8, 0.0], [-2.8, -4.0], [-2.8, 0.0], [-14.0, 0.0]]},
            {"id": 2, "name": "hospital_walking_human_2", "mode": "autonomous", "speed": 0.75, "scale": 0.95, "visual_z": 0.95, "proxy_z": 0.80, "waypoints": [[2.8, -4.0], [2.8, -1.1], [-19.6, -1.1], [-14.0, -1.1], [-14.0, 4.0], [-14.0, -1.1], [2.8, -1.1]]},
            {"id": 3, "name": "hospital_walking_human_3", "mode": "autonomous", "speed": 0.70, "scale": 1.02, "visual_z": 1.02, "proxy_z": 0.85, "waypoints": [[-21.0, -2.1], [-21.0, -1.1], [-14.0, -1.1], [-14.0, 4.0], [-14.0, -1.1], [-21.0, -1.1]]}
          ]
        })";
}

std::string GetHospitalMediumPresetJson()
{
  return R"({
          "episode_id": 1,
          "humans": [
            {"id": 1, "name": "medium_walker_01", "mode": "autonomous", "speed": 0.75, "scale": 1.00, "visual_z": 1.00, "proxy_z": 0.85, "waypoints": [[-10.0, 0.0], [-16.0, 0.0], [-17.0, -1.3], [-26.0, -1.3], [-17.0, -1.3], [-16.0, 0.0], [-10.0, 0.0]]},
            {"id": 2, "name": "medium_walker_02_head_on", "mode": "autonomous", "speed": 0.75, "scale": 0.95, "visual_z": 0.95, "proxy_z": 0.80, "waypoints": [[8.0, 0.0], [2.0, 0.0], [8.0, 0.0]]},
            {"id": 3, "name": "medium_walker_03_room_crossing", "mode": "autonomous", "speed": 0.70, "scale": 1.06, "visual_z": 1.06, "proxy_z": 0.875, "waypoints": [[-2.8, -4.0], [-2.8, 4.0], [-2.8, -4.0]]},
            {"id": 4, "name": "medium_walker_04_same_direction", "mode": "autonomous", "speed": 0.80, "scale": 1.00, "visual_z": 1.00, "proxy_z": 0.85, "waypoints": [[11.0, 0.0], [17.0, 0.0], [11.0, 0.0]]},
            {"id": 5, "name": "medium_walker_05_wait_then_cross", "mode": "autonomous", "speed": 0.65, "scale": 0.98, "visual_z": 0.98, "proxy_z": 0.82, "waypoints": [[19.6, -4.0], [19.6, 4.0], [19.6, -4.0]]},
            {"id": 6, "name": "medium_walker_06_trolley_pusher", "mode": "autonomous", "speed": 0.60, "scale": 1.00, "visual_z": 1.00, "proxy_z": 0.85, "waypoints": [[-24.0, 0.6], [-18.0, 0.6], [-24.0, 0.6]]},
            {"id": 7, "name": "medium_trolley_06", "mode": "stationary", "speed": 0.0, "scale": 1.00, "visual_z": 1.00, "proxy_z": 0.85, "waypoints": []},
            {"id": 8, "name": "medium_walker_07_yielding", "mode": "autonomous", "speed": 0.75, "scale": 1.02, "visual_z": 1.02, "proxy_z": 0.85, "waypoints": [[0.0, -1.35], [0.0, 1.35], [0.0, -1.35]]}
          ]
        })";
}

std::string GetHospitalHardPresetJson()
{
  return R"({
          "episode_id": 1,
          "humans": [
            {"id": 1, "name": "medium_walker_01", "mode": "autonomous", "speed": 0.75, "scale": 1.00, "visual_z": 1.00, "proxy_z": 0.85, "waypoints": [[-10.0, 0.0], [-16.0, 0.0], [-17.0, -1.3], [-26.0, -1.3], [-17.0, -1.3], [-16.0, 0.0], [-10.0, 0.0]]},
            {"id": 2, "name": "medium_walker_02_head_on", "mode": "autonomous", "speed": 0.75, "scale": 0.95, "visual_z": 0.95, "proxy_z": 0.80, "waypoints": [[8.0, 0.0], [2.0, 0.0], [8.0, 0.0]]},
            {"id": 3, "name": "medium_walker_03_room_crossing", "mode": "autonomous", "speed": 0.70, "scale": 1.06, "visual_z": 1.06, "proxy_z": 0.875, "waypoints": [[-2.8, -4.0], [-2.8, 4.0], [-2.8, -4.0]]},
            {"id": 4, "name": "medium_walker_04_same_direction", "mode": "autonomous", "speed": 0.80, "scale": 1.00, "visual_z": 1.00, "proxy_z": 0.85, "waypoints": [[11.0, 0.0], [17.0, 0.0], [11.0, 0.0]]},
            {"id": 5, "name": "medium_walker_05_wait_then_cross", "mode": "autonomous", "speed": 0.65, "scale": 0.98, "visual_z": 0.98, "proxy_z": 0.82, "waypoints": [[19.6, -4.0], [19.6, 4.0], [19.6, -4.0]]},
            {"id": 6, "name": "medium_walker_06_trolley_pusher", "mode": "autonomous", "speed": 0.60, "scale": 1.00, "visual_z": 1.00, "proxy_z": 0.85, "waypoints": [[-24.0, 0.6], [-18.0, 0.6], [-24.0, 0.6]]},
            {"id": 7, "name": "medium_trolley_06", "mode": "stationary", "speed": 0.0, "scale": 1.00, "visual_z": 1.00, "proxy_z": 0.85, "waypoints": []},
            {"id": 8, "name": "medium_walker_07_yielding", "mode": "autonomous", "speed": 0.75, "scale": 1.02, "visual_z": 1.02, "proxy_z": 0.85, "waypoints": [[0.0, -1.35], [0.0, 1.35], [0.0, -1.35]]},
            {"id": 9, "name": "hard_walker_08_door_exit", "mode": "autonomous", "speed": 0.75, "scale": 1.00, "visual_z": 1.00, "proxy_z": 0.85, "waypoints": [[-2.8, 4.0], [-2.8, 1.35], [-7.0, 1.35], [-2.8, 1.35], [-2.8, 4.0]]},
            {"id": 10, "name": "hard_walker_09_door_entry", "mode": "autonomous", "speed": 0.70, "scale": 0.96, "visual_z": 0.96, "proxy_z": 0.82, "waypoints": [[-7.0, -1.35], [-2.8, -1.35], [-2.8, -4.0], [-2.8, -1.35], [-7.0, -1.35]]},
            {"id": 11, "name": "hard_walker_10_cross_south_to_north", "mode": "autonomous", "speed": 0.80, "scale": 1.02, "visual_z": 1.02, "proxy_z": 0.85, "waypoints": [[2.8, -4.0], [2.8, 4.0], [2.8, -4.0]]},
            {"id": 12, "name": "hard_walker_11_cross_north_to_south", "mode": "autonomous", "speed": 0.80, "scale": 0.98, "visual_z": 0.98, "proxy_z": 0.82, "waypoints": [[8.4, 4.0], [8.4, -4.0], [8.4, 4.0]]},
            {"id": 13, "name": "hard_walker_12_group", "mode": "autonomous", "speed": 0.70, "scale": 1.04, "visual_z": 1.04, "proxy_z": 0.86, "waypoints": [[11.5, -0.4], [18.5, -0.4], [11.5, -0.4]]},
            {"id": 14, "name": "hard_walker_13_group", "mode": "autonomous", "speed": 0.70, "scale": 0.95, "visual_z": 0.95, "proxy_z": 0.80, "waypoints": [[11.5, 0.4], [18.5, 0.4], [11.5, 0.4]]},
            {"id": 15, "name": "hard_walker_14_trolley_pusher", "mode": "autonomous", "speed": 0.60, "scale": 1.00, "visual_z": 1.00, "proxy_z": 0.85, "waypoints": [[22.0, -1.2], [12.8, -1.2], [22.0, -1.2]]},
            {"id": 16, "name": "hard_trolley_14", "mode": "stationary", "speed": 0.0, "scale": 1.00, "visual_z": 1.00, "proxy_z": 0.85, "waypoints": []}
          ]
        })";
}

std::string GetSchoolArenaPresetJson()
{
  return R"({
          "episode_id": 1,
          "humans": [
            {"id": 1, "name": "student_head_on_A", "mode": "autonomous", "speed": 0.80, "scale": 1.02, "visual_z": 1.02, "proxy_z": 0.85, "waypoints": [[-12.0, -8.05], [10.0, -8.05], [-12.0, -8.05]]},
            {"id": 2, "name": "student_opposite_A", "mode": "autonomous", "speed": 0.75, "scale": 0.96, "visual_z": 0.96, "proxy_z": 0.82, "waypoints": [[10.0, -7.85], [-10.0, -7.85], [10.0, -7.85]]},
            {"id": 3, "name": "student_corner_B", "mode": "autonomous", "speed": 0.70, "scale": 1.04, "visual_z": 1.04, "proxy_z": 0.86, "waypoints": [[15.05, -7.0], [15.05, -4.1], [10.5, -4.0], [15.05, -4.1], [15.05, -7.0]]},
            {"id": 4, "name": "student_stair_loop", "mode": "autonomous", "speed": 0.75, "scale": 1.00, "visual_z": 1.00, "proxy_z": 0.85, "waypoints": [[14.85, -5.2], [15.0, -4.0], [7.0, -4.0], [7.0, 0.0], [14.9, 0.0], [15.0, 5.0], [14.9, 0.0], [7.0, 0.0], [7.0, -4.0], [15.0, -4.0], [14.85, -5.2]]},
            {"id": 5, "name": "student_door_exit", "mode": "autonomous", "speed": 0.65, "scale": 0.95, "visual_z": 0.95, "proxy_z": 0.80, "waypoints": [[-3.73, -10.4], [-3.73, -8.05], [2.0, -8.05], [-3.73, -8.05], [-3.73, -10.4]]},
            {"id": 6, "name": "student_vertical_up", "mode": "autonomous", "speed": 0.70, "scale": 1.02, "visual_z": 1.02, "proxy_z": 0.85, "waypoints": [[14.85, 1.4], [14.85, 12.4], [14.85, 1.4]]},
            {"id": 7, "name": "student_vertical_down", "mode": "autonomous", "speed": 0.70, "scale": 0.98, "visual_z": 0.98, "proxy_z": 0.82, "waypoints": [[15.15, 12.0], [15.15, 2.0], [15.15, 12.0]]},
            {"id": 8, "name": "student_short_cross", "mode": "autonomous", "speed": 0.60, "scale": 1.00, "visual_z": 1.00, "proxy_z": 0.85, "waypoints": [[15.15, 6.4], [14.65, 6.4], [15.35, 6.4], [15.15, 6.4]]}
          ]
        })";
}

std::string GetCorridorPresetJson(int humanCount, double corridorHalfWidth, double humanRadius)
{
  const double maxSpawnY = std::max(0.05, corridorHalfWidth - humanRadius - 0.02);
  const std::vector<double> yOffsets = {0.0, maxSpawnY * 0.5, -maxSpawnY * 0.5, maxSpawnY, -maxSpawnY};
  json root;
  root["episode_id"] = 1;
  root["corridor_mode"] = true;
  root["humans"] = json::array();
  for (int i = 1; i <= humanCount; ++i)
  {
    json hObj;
    hObj["id"] = i;
    hObj["mode"] = "autonomous";
    hObj["speed"] = 0.75;
    const double spawnX = 3.5 + (i - 1) * (7.0 / std::max(humanCount - 1, 1));
    const double spawnY = yOffsets[(i - 1) % yOffsets.size()];
    hObj["start"] = {{"x", spawnX}, {"y", spawnY}, {"yaw", 3.14159}};
    hObj["waypoints"] = json::array({{spawnX, spawnY}, {-5.0, spawnY}, {12.0, spawnY}});
    root["humans"].push_back(hObj);
  }
  return root.dump();
}

std::string GetDefaultPresetJson(int humanCount)
{
  if (humanCount == 1)
  {
    return R"({
        "episode_id": 1,
        "humans": [
          {"id": 1, "mode": "autonomous", "speed": 0.75, "scale": 1.02, "visual_z": 1.02, "proxy_z": 0.85, "waypoints": [[1.0, 0.0], [-6.0, 0.0]]}
        ]
      })";
  }

  return R"({
      "episode_id": 1,
      "humans": [
        {"id": 1, "mode": "autonomous", "speed": 0.75, "scale": 1.02, "visual_z": 1.02, "proxy_z": 0.85, "waypoints": [[4.0, 0.0], [-10.0, 0.0]]},
        {"id": 2, "mode": "autonomous", "speed": 0.85, "scale": 0.95, "visual_z": 0.95, "proxy_z": 0.80, "waypoints": [[-8.0, -1.2], [3.0, 1.0]]},
        {"id": 3, "mode": "autonomous", "speed": 0.70, "scale": 1.06, "visual_z": 1.06, "proxy_z": 0.875, "waypoints": [[-3.0, -3.0], [-3.0, 3.0]]},
        {"id": 4, "mode": "autonomous", "speed": 0.80, "scale": 1.00, "visual_z": 1.00, "proxy_z": 0.85, "waypoints": [[1.0, 2.5], [-7.0, -2.0]]}
      ]
    })";
}

void SyncTrolleyEntities(
    std::vector<HumanAgent> &humans)
{
  for (auto &human : humans)
  {
    if (human.name.find("trolley") != std::string::npos &&
        human.name.find("pusher") == std::string::npos)
    {
      for (const auto &pusher : humans)
      {
        if (pusher.name.find("trolley_pusher") != std::string::npos)
        {
          bool match = false;
          if (human.name.find("06") != std::string::npos && pusher.name.find("06") != std::string::npos) match = true;
          else if (human.name.find("14") != std::string::npos && pusher.name.find("14") != std::string::npos) match = true;
          else match = true;
          if (match)
          {
            human.x = pusher.x;
            human.y = pusher.y;
            human.yaw = pusher.yaw;
            human.animTimeSec = pusher.animTimeSec;
            human.animationTime = pusher.animationTime;
            break;
          }
        }
      }
    }
  }
}

} // namespace custom_corridor
