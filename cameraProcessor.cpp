#include "cameraProcessor.hpp"

#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

#ifdef FAST_CHECK
#define FIND_CB_FLAGS (CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK)
#else
#define FIND_CB_FLAGS (CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE)
#endif

#define DELTA_TIME 1

using namespace std;
using namespace cv;

static Size calib_fetch_data(int camera_index, vector<vector<Vec2f>>& img_points, Size pattern_size);
static void calib_compute_corners(vector<Vec3f>& corners, Size pattern_size, float square_size);
static void calib_save_params(Mat camera_matrix, Mat dist_coeffs, const char* path);

void camera_processor::calibrate_camera(int camera_index, 
                                        int pattern_width, int pattern_height, 
                                        float square_size, 
                                        const char* output_path) {

  vector<vector<Vec3f>> obj_points(1);
  vector<vector<Vec2f>> img_points; 

  Mat camera_matrix = Mat::eye(3, 3, CV_64F);
  Mat dist_coeffs   = Mat::zeros(5, 1, CV_64F);

  Size pattern_size = Size(pattern_width, pattern_height); 
  Size frame_size   = calib_fetch_data(camera_index, img_points, pattern_size);

  calib_compute_corners(obj_points[0], pattern_size, square_size);
  obj_points.resize(img_points.size(), obj_points[0]);
  vector<Mat> rvecs, tvecs;
  double rms = calibrateCamera(obj_points, img_points, frame_size, camera_matrix,
                               dist_coeffs, rvecs, tvecs);

  cout << "Final calibration error: " << rms << endl;

  calib_save_params(camera_matrix, dist_coeffs, output_path);
}



void camera_processor::undistort_camera(int camera_index, const char* camera_params_path, 
                                        const char* output_stream) {
  // TODO  
}

static Size calib_fetch_data(int camera_index, vector<vector<Vec2f>>& img_points, Size pattern_size) {
  VideoCapture cap;
    
  if(!cap.open(camera_index)) {
    cerr << "Could not open camera with id " << camera_index << '!' << endl; 
    exit(EXIT_FAILURE);
  }

  cout << "Press 'q' to quit the calibration process!" << endl;

  auto lastTime = chrono::system_clock::now();
  
  Mat frame, gray;
  while(true) {
    cap >> frame;
    if( frame.empty() ) {
      cerr << "Error loading frame! " << endl; 
      exit(EXIT_FAILURE);
    }
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    vector<Vec2f> corners;
    
    if(findChessboardCorners(gray, pattern_size, corners, FIND_CB_FLAGS)) {
      cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                 TermCriteria(TermCriteria::Type::EPS + TermCriteria::Type::COUNT, 30,1e-3));   
          
      drawChessboardCorners(frame, pattern_size, corners, true);
          
      auto currentTime = chrono::system_clock::now();
      double deltaTime = chrono::duration<double>(currentTime - lastTime).count();
 
      if(deltaTime > DELTA_TIME) {
        img_points.push_back(corners);
        lastTime = currentTime;
        cout << "Image captured! " << img_points.size() << " total images!\n";
      }
    }
    
    imshow("Calibrate camera", frame); 
    if( waitKey(10) == 'q' ) {
      break; 
    }
  }
  destroyAllWindows();
  return frame.size();
}


static void calib_compute_corners(vector<Vec3f>& corners, Size pattern_size, float square_size) {
  corners.clear();

  for(int i = 0; i < pattern_size.height; i++) {
    for( int j = 0; j < pattern_size.width; j++) {
      corners.push_back(Point3f(float( j*square_size ), float( i*square_size ), 0));
    }
  }
}

static void calib_save_params(Mat camera_matrix, Mat dist_coeffs, const char* path) {
  ofstream out(path);

  out << camera_matrix << dist_coeffs;

  out.close();
}
