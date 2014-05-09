
#include <iostream>
#include <iomanip>
#include <thread>
#include "cameraPoseEstimator.h"

void dataCheck(CameraPoseEstimator* camPose) {
  	Pose3D pose;
    while(true) {
    	if(camPose->dataAvailable()) {
    	  camPose->getPose(pose);
    	  std::cout << "POSE: " << pose << std::endl;
        }
    	std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main() {
	CameraPoseEstimator camPose;

    std::thread runCam(&CameraPoseEstimator::continuousRead, &camPose);
    std::thread getData(&dataCheck, &camPose);
    runCam.join();
    getData.join();

	return 0;
}
