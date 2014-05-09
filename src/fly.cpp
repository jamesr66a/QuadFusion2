#include <thread>
#include <chrono>
#include <iostream>

#include "optical_flow.h"
#include "cameraPoseEstimator.h"

int main()
{
    CameraPoseEstimator cpe;
    OpticalFlowSensor ofs;

    std::thread cpe_thread(&CameraPoseEstimator::continuousRead, &cpe);
    std::thread ofs_thread(&OpticalFlowSensor::loop, &ofs, "/dev/ttyO0");
	
    auto timestamp = std::chrono::high_resolution_clock::now();

    while (true)
    {
	FlowData fd;
	Pose3D pose;
	
	float x, y = 0;

        if (ofs.dataReady())
	{
	    fd = ofs.getFlowData();
	//    std::cout << fd.flow_x << " "
	//	      << fd.flow_y << " "
	//	      << std::endl;
	
	auto temp_stamp = std::chrono::high_resolution_clock::now();
	auto duration = temp_stamp - timestamp;

	x += fd.flow_x*duration.count();
	y += fd.flow_y*duration.count();
	std::cout << x << " " << y << std::endl;
	}
	if (cpe.dataAvailable())
	{
	    cpe.getPose(pose);
	//    std::cout << pose << std::endl;
	x = pose.x;
	y = pose.y;
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
