#ifndef CAMERA_PROCESSOR_HPP
#define CAMERA_PROCESSOR_HPP

namespace camera_processor {
  /* Perform camera calibration
   * camera_index   = index of camera to calibrate
   * pattern_width  = width of the chessboard calibration pattern
   * pattern_height = height of the chessboard calibration pattern
   * square_size    = length of the side of one square in the calibration pattern
   * output_path    = path to the file where results should be saved
   */
  void calibrate_camera(unsigned int camera_index, 
                        unsigned int pattern_width, unsigned int pattern_height, 
                        float square_size, 
                        const char* output_path);


  /* Perform camera calibration
   * camera_index       = index of camera to undistort
   * camera_params_path = path to the file containing the camera parameters
   * output_stream      = path to video device where the undistorted video should be streamd
   */
  void undistort_camera(unsigned int camera_index, 
                        const char* camera_params_path, 
                        const char* output_stream);
};

#endif // CAMERA_PROCESSOR_HPP

