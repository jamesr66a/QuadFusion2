#include <thread>
#include <chrono>
#include <iostream>


#include "optical_flow.h"
#include "cameraPoseEstimator.h"
#include "PID.h"
#include "GPIO.h"


int main()
{
    CameraPoseEstimator cpe;
    OpticalFlowSensor ofs;

    std::thread cpe_thread(&CameraPoseEstimator::continuousRead, &cpe);
    std::thread ofs_thread(&OpticalFlowSensor::loop, &ofs, "/dev/ttyO0");
	
    float x = 0, y = 0;
    float setPointX=0, setPointY=0, setPointZ=0;
    bool prevFlightStatus=false;
    float pidRoll, pidPitch, pidThrottle;

    
    initGPIO(16,false);
    writeGpio(16, true);
    init GPIO(17,true);

    while (true)
    {
        if(readGPIO(17)=='1') inFlight=true;
        if(readGPIO(17)=='0') inFlight=false;
        
	if(prevFlightStatus!=inFlight){
		setPointX=x;
		setPointY=y;

	}
	
	prevFlightStatus=inFlight;
	

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
