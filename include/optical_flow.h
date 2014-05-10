#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include <string>
#include <vector>
#include <stdint.h>
#include <mutex>
#include <queue>
#include <atomic>

struct FlowData{
    float flow_x;
    float flow_y;
    float ground_distance;
    FlowData(float flow_x, float flow_y, float ground_distance)
        : flow_x(flow_x), flow_y(flow_y), ground_distance(ground_distance)
    {}
	FlowData() {}
};

class OpticalFlowSensor
{
public:
    void loop(std::string device_file);
    bool dataReady();
    FlowData getFlowData();
	OpticalFlowSensor()
	{
		ready.store(false);
	}
private:
	std::atomic<bool> ready;
	std::mutex data_points_mutex;

	std::queue<FlowData> data_points;
	
	int fd;

	int set_interface_attribs(int fd, int speed, int parity);
};

#endif
