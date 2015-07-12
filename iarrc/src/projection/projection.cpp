#include <opencv/cv.h>
#include "math.h"
#include "projection.h"

using namespace cv; 
namespace proj {

    void groundTransformProj(const Mat& input, const ProjectionParams& params, Mat& output) {

        // resolution of the input image in the camera frame
        const Size input_size = input.size();
        const int input_x_res = input_size.width;
        const int input_y_res = input_size.height;

        // resolution of the output image in the ground frame
        const int output_x_res = params.output_x_res;
        const int output_y_res = params.output_y_res;

        const double ground_output_x_dim = params.ground_output_x_dim;
        const double ground_output_y_dim = params.ground_output_y_dim;
        const double ground_output_z_dim = params.ground_output_z_dim;

        const double camera_scale = params.camera_scale; 
        const double camera_pitch = params.camera_pitch;

        output = Mat::zeros(Size(output_x_res, output_y_res), input.type());
        // compute trig outside of the loop for such fastness
        const double cos_camera_pitch = cos(camera_pitch);
        const double sin_camera_pitch = sin(camera_pitch);
        
        for (int output_x = 0; output_x < output_x_res; output_x++) {
            for (int output_y = 0; output_y < output_y_res; output_y++) {

                // ground frame coordinates of sample point (in ft, it's all relative)
                const double x_ground = ((double) output_x / output_x_res) * ground_output_x_dim - (ground_output_x_dim / 2);
                const double y_ground = ((double) output_y / output_y_res) * ground_output_y_dim;
                const double z_ground = -ground_output_z_dim;

                // camera frame (post-transform) coordinates of ground sample point
                const double x_camera = x_ground;
                const double y_camera = y_ground * cos_camera_pitch - z_ground * sin_camera_pitch;
                const double z_camera = y_ground * sin_camera_pitch + z_ground * cos_camera_pitch;

                if (z_camera <= 0.0) {
                    // don't divide by zero or look behind the camera
                    continue;
                }

                // input image coordinates
                const int input_x = camera_scale * (x_camera / z_camera) + (input_x_res / 2);
                const int input_y = camera_scale * (y_camera / z_camera) + (input_y_res / 2);
                
                if (input_y < 0 || input_y >= input_y_res || input_x < 0 || input_x >= input_x_res) {
                    // don't sample outside the camera frame
                    continue;
                }
               
                if (input.channels() == 3) {
                    output.at<Vec3b>(output_y_res - output_y, output_x) =
                        input.at<Vec3b>(input_y, input_x);
                } else {
                    output.at<unsigned char>(output_y_res - output_y, output_x_res - output_x) =
                        input.at<unsigned char>(input_y, input_x);
                }
            }
        }
    }
}