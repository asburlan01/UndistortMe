# UndistortMe
Undistort video output from webcams in Linux. <br><br>
Many webcams add visible distortion to the video output. This project aims to solve this issue by providing the means to calibrate such a camera and then use the newly found camera parameters to produce undistorted videos. The output is streamed to the selected v4l2loopback device which can be subsequently opened in other apps. <br><br>
Requirements: OpenCV 4.0, v4l2loopback <br><br>
Usage: <br>
  - undistortMe calibrate camera_index pattern_width pattern_height square_size camera_params <br>
    Calibrate the camera with id camera_index using a chessboard of size <br>
 (pattern_width x pattern_height) with squares of size square_size <br>
 and place the output params in the file camera_params <br>
  - undistortMe undistort camera_index camera_params output_stream <br>
    Run the undistort algorithm on the camera with id camera_index using parameters in camera_params <br>
 and stream the output to output_stream. Output stream should be a virtual video device created using <br>
 v4l2loopback <br>
