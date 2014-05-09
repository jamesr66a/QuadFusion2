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
	
    float x = 0, y = 0;
   
    while (true)
    {
	FlowData fd;
	Pose3D pose;
	
        if (ofs.dataReady())
	{
	    fd = ofs.getFlowData();
	    //std::cout << fd.flow_x << " "
	    //	      << fd.flow_y << " "
	    //	      << std::endl;
	    x += fd.flow_x;
	    y += fd.flow_y;
	    std::cout << x << " " << y << std::endl;
	    //std::cout << std::chrono::nanoseconds(dt).count()/10e6 << std::endl;
	}
	if (cpe.dataAvailable())
	{
	    cpe.getPose(pose);
	
	    x = pose.x;
	    y = pose.y;
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
