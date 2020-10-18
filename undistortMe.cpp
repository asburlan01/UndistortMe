#include <iostream>
#include <cstring>

#include "cameraProcessor.hpp"

using namespace std;
using namespace camera_processor;

void print_usage(ostream& out_str) {
  out_str << "Usage: " << endl;
  out_str << "  - undistortMe calibrate camera_index camera_params" << endl;
  out_str << "    Calibrate the camera with id camera_index and place the output params in camera_params" << endl;
  out_str << "  - undistortMe undistort camera_index camera_params output_stream" << endl;
  out_str << "    Run the undistort algorithm on the camera with id camera_index using parameters in camera_params"  << endl
       << " and stream the output to output_stream. Output stream should be a virtual video device created using" << endl
       << " v4l2loopback " << endl;
}

int main(int argc, char** argv) {
  if(argc < 2) {
    cerr << "Not enough arguments!" << endl;
    print_usage(cerr);
    return EXIT_SUCCESS;
  }

  if(strcmp(argv[1], "calibrate") == 0) {
    if(argc != 4) {
      cerr << "Invalid arguments!" << endl;
      print_usage(cerr);
      return EXIT_SUCCESS;
    }
    int index = atoi(argv[2]);
    
    calibrate_camera(index, argv[3]);
  } else if(strcmp(argv[1], "undistort") == 0) {
    if(argc != 5) {
      cerr << "Invalid arguments!" << endl;
      print_usage(cerr);
      return EXIT_SUCCESS;
    }
    int index = atoi(argv[2]);
    
    undistort_camera(index, argv[3], argv[4]);
  } else {
    cerr << "Unsupported operation!" << endl;
    print_usage(cerr);
    return EXIT_FAILURE;
  }  
    
  return EXIT_SUCCESS;
}
