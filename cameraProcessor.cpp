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

static Size calib_fetch_data(const int camera_index, const Size pattern_size, vector<vector<Vec2f>>& img_points);
static void calib_compute_corners(const Size pattern_size, const float square_size, vector<Vec3f>& corners);
static void calib_save_params(const Mat& camera_matrix, const Mat& dist_coeffs, const char* path);

static void undistort_load_params(Mat& camera_matrix, Mat& dist_coeffs, const char* path);
static int v4l2_open_stream(const char* path, const int width, const int height, const int frame_bytes); 
static void print_mat_to_stream(const int device, const Mat& img, const int frame_bytes);

void camera_processor::calibrate_camera(const unsigned int camera_index, 
                                        const unsigned int pattern_width, const unsigned int pattern_height, 
                                        const float square_size, 
                                        const char* output_path) {

  assert(output_path);

  vector<vector<Vec3f>> obj_points(1); // calib point coords in the chessboard plane reference frame
  vector<vector<Vec2f>> img_points;    // calib point coords in the image plane

  Mat camera_matrix = Mat::eye(3, 3, CV_64F);
  Mat dist_coeffs   = Mat::zeros(8, 1, CV_64F);

  Size pattern_size = Size(pattern_width, pattern_height);                      // size of the calibration pattern
  Size frame_size   = calib_fetch_data(camera_index, pattern_size, img_points); // size of one frame
  if(img_points.empty()) {
    cerr << "No calibration images were taken! Quitting..." << endl;
    exit(EXIT_FAILURE);
  }

  calib_compute_corners(pattern_size, square_size, obj_points[0]);
  obj_points.resize(img_points.size(), obj_points[0]);
  vector<Mat> rvecs, tvecs;
  double rms = calibrateCamera(obj_points, img_points, frame_size, camera_matrix,
                               dist_coeffs, rvecs, tvecs);

  cout << "Final calibration error: " << rms << endl;

  calib_save_params(camera_matrix, dist_coeffs, output_path);
}

void camera_processor::undistort_camera(const unsigned int camera_index, 
                                        const char* camera_params_path, 
                                        const char* output_stream) {
  VideoCapture cap;
    
  if(!cap.open(camera_index)) {
    cerr << "Could not open camera with id " << camera_index << '!' << endl; 
    exit(EXIT_FAILURE);
  }

  Mat camera_matrix = Mat::eye(3, 3, CV_64F);
  Mat dist_coeffs   = Mat::zeros(8, 1, CV_64F);
  undistort_load_params(camera_matrix, dist_coeffs, camera_params_path);

  int width  = cap.get(CAP_PROP_FRAME_WIDTH);  // frame width
  int height = cap.get(CAP_PROP_FRAME_HEIGHT); // frame height
  int frame_bytes = 3 * width * height / 2;    // number of bytes in a YUV420 frame

  int stream = v4l2_open_stream(output_stream, width, height, frame_bytes); // output stream
  
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

/* Fetch data required for camera calibration
 * camera_index = index of camera to calibrate
 * pattern_size = size of the chessboard calibration pattern
 * img_points   = reference to the vector where results should be placed
 *
 * returns the size of a frame used for calibration
 */
static Size calib_fetch_data(const int camera_index, const Size pattern_size, vector<vector<Vec2f>>& img_points) {
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
    
    if(findChessboardCorners(gray, pattern_size, corners, FIND_CB_FLAGS)) {  // if the pattern was found
      // Refine the point coordonates. Using parameters recommended on the opencv website
      cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                 TermCriteria(TermCriteria::Type::EPS + TermCriteria::Type::COUNT, 30,1e-3));   
      
      // Draw results to the screen
      drawChessboardCorners(frame, pattern_size, corners, true);
          
      auto currentTime = chrono::system_clock::now();
      double deltaTime = chrono::duration<double>(currentTime - lastTime).count();
 
      if(deltaTime > DELTA_TIME) {  // Fetch data only if at least DELTA_TIME seconds have passed
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

/* Compute the coordonates of the calibration pattern points in the coordonate frame of the chessboard
 * where all points lie in the XY plane.
 * pattern_size = size of the calibration pattern
 * square_size  = length of the side of one square
 * corners      = output array of the generated coordonates
 */
static void calib_compute_corners(const Size pattern_size, const float square_size, vector<Vec3f>& corners) {
  corners.clear();

  for(int i = 0; i < pattern_size.height; i++) {
    for( int j = 0; j < pattern_size.width; j++) {
      corners.push_back(Point3f(float( j*square_size ), float( i*square_size ), 0));
    }
  }
}

// Print the given Mat to the given output stream
static void print_mat(ostream& out, const Mat& m) {
  for(int i = 0; i < m.rows; i++) {
    for(int j = 0; j < m.cols; j++) {
      out << m.at<double>(i,j) << ' ';
    }
    out << '\n';
  } 
}

// Read data into the given Mat from the given input stream
static void read_mat(istream& in, Mat& m) {
  for(int i = 0; i < m.rows; i++) {
    for(int j = 0; j < m.cols; j++) {
      in >> m.at<double>(i,j);
    }
  } 
}

// Save calibration params
static void calib_save_params(const Mat& camera_matrix, const Mat& dist_coeffs, const char* path) {
  ofstream out(path);
  
  print_mat(out, camera_matrix);
  print_mat(out, dist_coeffs);
  
  out.close();
}

// Load parameters for undistortion algorithm
static void undistort_load_params(Mat& camera_matrix, Mat& dist_coeffs, const char* path) { 
  ifstream in(path);

  read_mat(in, camera_matrix);
  read_mat(in, dist_coeffs);

  in.close();
}

/* Open a v4l2loopback video device for streaming data
 * path        = path to device
 * width       = width of the image
 * height      = height of the image
 * frame_bytes = number of bytes in a frame
 *
 * returns a handle to the video device
 *
 * Code inspired from the official v4l2loopback examples on github
 * https://github.com/umlaeute/v4l2loopback
 */
static int v4l2_open_stream(const char* path, const int width, const int height, const int frame_bytes) { 
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

// Print frame_bytes bytes from the given Mat to the given v4l2loopback device
static void print_mat_to_stream(const int device, const Mat& img, const int frame_bytes) {
  Mat yuv;
  cvtColor(img, yuv, COLOR_BGR2YUV_I420);
  write(device, yuv.ptr(), frame_bytes);
}
