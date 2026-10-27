#pragma once

#include <string>
#include <vector>
#include "custom_corridor_plugins/types.hpp"

namespace custom_corridor
{

std::string GetHospitalEasyPresetJson();
std::string GetHospitalMediumPresetJson();
std::string GetHospitalHardPresetJson();
std::string GetSchoolArenaPresetJson();
std::string GetCorridorPresetJson(int humanCount, double corridorHalfWidth, double humanRadius);
std::string GetDefaultPresetJson(int humanCount);

void SyncTrolleyEntities(
    std::vector<HumanAgent> &humans);

} // namespace custom_corridor
