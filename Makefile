CXX=g++
LDLIBS= -lstdc++

undistortMe: undistortMe.o cameraProcessor.o

undistortMe.o: cameraProcessor.hpp

cameraProcessor.o: cameraProcessor.hpp

.PHONY: clean
clean:
	rm -f undistortMe undistortMe.o cameraProcessor.o
