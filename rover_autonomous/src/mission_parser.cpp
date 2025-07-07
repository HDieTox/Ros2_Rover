#include "rover_autonomous/mission_parser.hpp"
#include <fstream>
#include <stdexcept>

using json = nlohmann::json;

std::vector<MissionPoint> MissionParser::parse_mission(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        throw std::runtime_error("Impossible d'ouvrir le fichier: " + file_path);
    }

    json mission_data;
    file >> mission_data;

    std::vector<MissionPoint> points;
    for (const auto& item : mission_data["mission"]["items"]) {
        if (item["command"] == 16) {  // MAV_CMD_NAV_WAYPOINT
            MissionPoint pt{
                item["params"][4],
                item["params"][5],
                item["params"][6],
                item["params"][2]  // Acceptance radius
            };
            points.push_back(pt);
        }
    }
    return points;
}