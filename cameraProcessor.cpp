#include "cameraProcessor.hpp"

#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <cstdlib>

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>

#include <cassert>

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

static void undistort_load_params(Mat& camera_matrix, Mat& dist_coeffs, const char* path);
static int open_stream(const char* path, int width, int height, int frame_bytes);
static void print_mat_to_stream(int stream, Mat& img, int frame_bytes);

void camera_processor::calibrate_camera(int camera_index, 
                                        int pattern_width, int pattern_height, 
                                        float square_size, 
                                        const char* output_path) {

  assert(output_path);

  vector<vector<Vec3f>> obj_points(1);
  vector<vector<Vec2f>> img_points; 

  Mat camera_matrix = Mat::eye(3, 3, CV_64F);
  Mat dist_coeffs   = Mat::zeros(8, 1, CV_64F);

  Size pattern_size = Size(pattern_width, pattern_height); 
  Size frame_size   = calib_fetch_data(camera_index, img_points, pattern_size);
  if(img_points.empty()) {
    cerr << "No calibration images were taken! Quitting..." << endl;
    exit(EXIT_FAILURE);
  }

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
  VideoCapture cap;
    
  if(!cap.open(camera_index)) {
    cerr << "Could not open camera with id " << camera_index << '!' << endl; 
    exit(EXIT_FAILURE);
  }

  Mat camera_matrix = Mat::eye(3, 3, CV_64F);
  Mat dist_coeffs   = Mat::zeros(8, 1, CV_64F);
  undistort_load_params(camera_matrix, dist_coeffs, camera_params_path);

  int width = cap.get(CAP_PROP_FRAME_WIDTH);
  int height = cap.get(CAP_PROP_FRAME_HEIGHT);
  int frame_bytes = 3 * width * height / 2;

  int stream = open_stream(output_stream, width, height, frame_bytes);
  
  cout << "Press 'q' to close the virtual camera!" << endl;
  Mat frame, gray;
  while(true) {
    cap >> frame;
    if( frame.empty() ) {
      cerr << "Error loading frame! " << endl; 
      close(stream);
      exit(EXIT_FAILURE);
    }
    Mat undistorted_frame = frame.clone();
    undistort(frame, undistorted_frame, camera_matrix, dist_coeffs);
    print_mat_to_stream(stream, undistorted_frame, frame_bytes);
    
    imshow("Undistorted Preview", undistorted_frame); 
    undistorted_frame.release();
    if( waitKey(10) == 'q' ) {
      break; 
    }
  }
  close(stream);
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
    
    imshow("Calibrate Camera", frame); 
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

static void print_mat(ostream& out, Mat& m) {
  for(int i = 0; i < m.rows; i++) {
    for(int j = 0; j < m.cols; j++) {
      out << m.at<double>(i,j) << ' ';
    }
    out << '\n';
  } 
}

static void read_mat(istream& in, Mat& m) {
  for(int i = 0; i < m.rows; i++) {
    for(int j = 0; j < m.cols; j++) {
      in >> m.at<double>(i,j);
    }
  } 
}

static void calib_save_params(Mat camera_matrix, Mat dist_coeffs, const char* path) {
  ofstream out(path);
  
  print_mat(out, camera_matrix);
  print_mat(out, dist_coeffs);
  
  out.close();
}

static void undistort_load_params(Mat& camera_matrix, Mat& dist_coeffs, const char* path) { 
  ifstream in(path);

  read_mat(in, camera_matrix);
  read_mat(in, dist_coeffs);

  in.close();
}

static int open_stream(const char* path, int width, int height, int frame_bytes) { 
  int stream = open(path, O_RDWR);
  if(stream == -1) {
    cerr << "Could not open output stream!" << endl;
    exit(EXIT_FAILURE);
  }
  
  struct v4l2_format v;
  memset(&v, 0, sizeof(v));
  
  v.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  if (ioctl(stream, VIDIOC_G_FMT, &v) == -1){
    cerr << "Could not setup video device" << endl;
    close(stream);
    exit(EXIT_FAILURE);
  }
  v.fmt.pix.width = width;
  v.fmt.pix.height = height;
  v.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
  v.fmt.pix.sizeimage = frame_bytes;
  v.fmt.pix.field = V4L2_FIELD_NONE;
  
  if (ioctl(stream, VIDIOC_S_FMT, &v) == -1){
    cerr << "Could not setup video device" << endl;
    close(stream);
    exit(EXIT_FAILURE);
  }
  return stream;
}

static void print_mat_to_stream(int stream, Mat& img, int frame_bytes) {
  Mat yuv;
  cvtColor(img, yuv, COLOR_BGR2YUV_I420);
  write(stream, yuv.ptr(), frame_bytes);
}
