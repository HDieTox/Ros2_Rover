#pragma once

#include <string>
#include <vector>
#include "nlohmann/json.hpp"

struct MissionPoint {
    double latitude;
    double longitude;
    double altitude;
    double tolerance;  // Tolerance in metres
};

class MissionParser {
public:
    /**
     * Charge un fichier de mission QGroundControl (.plan)
     * 
     * @param file_path Chemin vers le fichier .plan
     * @return Vecteur des points de mission
     * @throws std::runtime_error en cas d'erreur de parsing
     */
    static std::vector<MissionPoint> parse_mission(const std::string& file_path);
};