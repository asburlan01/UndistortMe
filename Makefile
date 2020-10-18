CXX=g++
CXXFLAGS=-I/usr/include/opencv4
LDLIBS= -lstdc++ -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_videoio -lopencv_calib3d

undistortMe: undistortMe.o cameraProcessor.o

undistortMe.o: cameraProcessor.hpp

cameraProcessor.o: cameraProcessor.hpp

.PHONY: clean
clean:
	rm -f undistortMe undistortMe.o cameraProcessor.o
