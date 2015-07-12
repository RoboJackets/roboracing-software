#ifndef __PROJECTION_H
#define __PROJECTION_H

using namespace cv;
namespace proj {
    struct ProjectionParams {
    // output image resolution
    int output_x_res;
    int output_y_res;

        // dimensions of the ground frame we are sampling from
        // ground frame is the rectangle from
        //   (-ground_output_x_dim / 2, 0)
        //   to (ground_output_x_dim, ground_output_y_dim)
        //   i.e. (-15 ft, 0 ft) to (15 ft, 30 ft)
    //   ground_output_z_dim is height of camera from the ground
    double ground_output_x_dim; //meters
    double ground_output_y_dim; //meters
    double ground_output_z_dim; //meters
        // general scale factor for the camera frame the input is in
    double camera_scale; //pixels/focal-length
    double camera_pitch; //radians from vertical

        ProjectionParams(int output_x_res, int output_y_res, double ground_output_x_dim, double ground_output_y_dim, double ground_output_z_dim, double camera_scale, double camera_pitch)
         : output_x_res(output_x_res), output_y_res(output_y_res), ground_output_x_dim(ground_output_x_dim), ground_output_y_dim(ground_output_y_dim), ground_output_z_dim(ground_output_z_dim), camera_scale(camera_scale), camera_pitch(camera_pitch) {}

    };

    void groundTransformProj(const Mat& input, const ProjectionParams& params, Mat& output);
}
#endif //__PROJECTION_H