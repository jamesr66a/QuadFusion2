#include <thread>
#include <chrono>
#include <iostream>


#include "optical_flow.h"
#include "cameraPoseEstimator.h"
#include "PID.h"
#include "GPIO.h"

float constain(float a, float x, float y);
void initPID();

int main()
{
    CameraPoseEstimator cpe;
    OpticalFlowSensor ofs;

    std::thread cpe_thread(&CameraPoseEstimator::continuousRead, &cpe);
    std::thread ofs_thread(&OpticalFlowSensor::loop, &ofs, "/dev/ttyO0");
	
    float x = 0, y = 0;
    float setPointX=0, setPointY=0, setPointZ=0;
    bool prevFlightStatus=false, firstRead=true;
    float pidRoll, pidPitch, pidThrottle;

    
    initPID();

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
	    std::cout << x << " " << y << " " << gd.ground_distance << std::endl;
	    //std::cout << std::chrono::nanoseconds(dt).count()/10e6 << std::endl;
	}
	if (cpe.dataAvailable())
	{
	    cpe.getPose(pose);
	
	    x = pose.x;
   	    y = pose.y;
	    
            if(firstRead){
		
	        currentTime=getCurrentTime();

		firstRead=false;

	    }
	    pidPitch=updatePID(setPointX,x,&PID[PITCH]);
            pidRoll=updatePID(setPointY,y,&PID[ROLL]);

	    /*TODO: get PID for throttle and then do
	     1500+ constrain stuff. 
	     figure it out James, I'm tired now.

		-Griffin

	    */	    
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
