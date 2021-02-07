//
// Created by charlie on 2/1/21.
//

#pragma once

struct april_tag {
    int id;
    double size;
    double x;
    double y;
    double z;
    double qw;
    double qx;
    double qy;
    double qz;
};

struct april_robot {
    std::string name;
    std::unordered_map<int, april_tag> tags;
};
