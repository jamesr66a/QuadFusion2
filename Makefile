optical_flow: src/optical_flow.cpp include/optical_flow.h
	cpp --std=c++11 -Iinclude -o tmp/optical_flow.cpp src/optical_flow.cpp
	armv7a-hardfloat-linux-gnueabi-c++ -std=c++11 -Iinclude -o obj/optical_flow.o -c tmp/optical_flow.cpp

clean:
	rm obj/* fly tmp/*

TAG_INCLUDE_FILES=-I/usr/include -I/usr/local/include
TAG_LIBRARY_FILES=-L/usr/local/lib
   
TAG_LIBS=-lopencv_core\
      -lopencv_calib3d\
      -lopencv_contrib\
      -lopencv_features2d\
      -lopencv_flann\
      -lopencv_gpu\
      -lopencv_highgui\
      -lopencv_imgproc\
      -lopencv_legacy\
      -lopencv_ml\
      -lopencv_objdetect\
      -lopencv_video\
      -lopencv_nonfree

fly: src/fly.cpp include/SquarePattern.h include/StoredPatterns.h include/cameraPoseEstimator.h include/econ.h include/findPose.h optical_flow
	g++ --std=c++11 -Iinclude $(TAG_INCLUDE_FILES) $(TAG_LIBRARY_FILES) -o ./fly src/fly.cpp obj/optical_flow.o  $(TAG_LIBS) -lpthread
