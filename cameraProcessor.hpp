#ifndef CAMERA_PROCESSOR_HPP
#define CAMERA_PROCESSOR_HPP

namespace camera_processor {
  void calibrate_camera(int camera_index, const char* output_path);

  void undistort_camera(int camera_index, const char* camera_params_path, 
                                          const char* output_stream);
};

#endif // CAMERA_PROCESSOR_HPP
