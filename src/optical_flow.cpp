#include "optical_flow.h"

#include <mavlink/common/mavlink.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <signal.h>
#include <stdlib.h>
#include <string>
#include <queue>
#include <thread>
#include <chrono>
#include <iostream>

bool OpticalFlowSensor::dataReady()
{
    return ready.load();
}

FlowData OpticalFlowSensor::getFlowData()
{
    data_points_mutex.lock();
    FlowData ret_val = data_points.front();
    data_points.pop();
    if (data_points.empty())
        ready.store(false);
    data_points_mutex.unlock();

    return ret_val;
}

void OpticalFlowSensor::loop(std::string device_file)
{
    char buf[10];
    mavlink_message_t msg;
    mavlink_status_t status;

    //signal(SIGINT, sig_handler);    

    fd = open(device_file.c_str(), O_RDWR | O_SYNC | O_NOCTTY);
    set_interface_attribs(fd, B115200, 0);

    float flow_x = 0, temp_flow_x = 0;
    float flow_y = 0, temp_flow_y = 0;
    float ground_distance = 0;    
    uint8_t history = 0;
    uint64_t timestamp = -1, base_timestamp = -1;
    while (1)
    {
        read(fd, buf, 1);

        if (mavlink_parse_char(0, buf[0], &msg, &status))
        {
		if (history++ == 0)
		{
			base_timestamp = mavlink_msg_optical_flow_get_time_usec(&msg);
		}
		else
		{
			timestamp = mavlink_msg_optical_flow_get_time_usec(&msg);
			flow_x = mavlink_msg_optical_flow_get_flow_comp_m_x(&msg);
			flow_y = mavlink_msg_optical_flow_get_flow_comp_m_y(&msg);
			ground_distance = mavlink_msg_optical_flow_get_ground_distance(&msg);
            data_points_mutex.lock();
			data_points.push(FlowData(flow_x, flow_y, ground_distance));
			ready.store(true);
			data_points_mutex.unlock();
			history = 0;
        	}
	}
	//std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}


int OpticalFlowSensor::set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        //error_message ("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // ignore break signal
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        //error_message ("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}
