#include <string>
#include <stdio.h>

double extractSpeed(std::string s) {
    // v=$float I=$float
    double speed = 0.0;
    // // Get interval from 'v=$' up to the next space. Must add 2 to include to = and include the $
    std::string speed_str(s.begin() + s.find('=') + 2, s.begin() + s.find(' '));
    if (!speed_str.empty()) {
        printf("value: %s\n", speed_str.c_str());
        speed = stod(speed_str);
    }
    return speed;
}

double extractSteering(std::string a) {
    // A=$float
    double angle = 0.0;
    std::string angle_str(a.begin() + a.find('=') + 2, a.end());
    if (!angle_str.empty()) {
        angle = stod(angle_str);
    }
    return angle;
}

int main(int argc, char const *argv[])
{
    std::string val = "A=$30.1";
    printf("steering: %f\n", extractSteering(val));
    return 0;
}
