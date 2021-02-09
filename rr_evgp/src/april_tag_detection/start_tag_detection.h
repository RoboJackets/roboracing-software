//
// Created by charlie on 2/1/21.
//

#pragma once

struct april_tag {
    int id;
    double size;
    tf::Pose pose;
};

struct april_robot {
    std::string name;
    std::unordered_map<int, april_tag> tags;
};
