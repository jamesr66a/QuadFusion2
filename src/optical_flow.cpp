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
    uint8_t history = 0;
    uint64_t timestamp = -1;
    while (1)
    {
        read(fd, buf, 1);

        if (mavlink_parse_char(0, buf[0], &msg, &status))
        {
#ifdef DEBUG
            printf("packet received\n");
            printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
#endif

            if (msg.msgid == 100) {
                if (history == 0)
		{
			timestamp = mavlink_msg_optical_flow_get_time_usec(&msg);	
			//std::cout << timestamp << std::endl;
		}
		else
		{
			uint64_t base_timestamp = timestamp;
			timestamp = mavlink_msg_optical_flow_get_time_usec(&msg);
			temp_flow_x = ((mavlink_msg_optical_flow_get_flow_comp_m_x(&msg)/(timestamp - base_timestamp))*10e8);
                	temp_flow_y = ((mavlink_msg_optical_flow_get_flow_comp_m_y(&msg)/(timestamp-base_timestamp))*10e8);
			if (history == 2)
			{
				flow_x = temp_flow_x;
				flow_y = temp_flow_y;
			}
			else
			{
				flow_x = (flow_x + temp_flow_x)/2;
				flow_y = (flow_y + temp_flow_y)/2;
			}
		}

                history++;
#ifdef DEBUG
                printf("%i\n", history);
#endif
                if (history == 4) {
                    data_points_mutex.lock();
                    data_points.push(FlowData(flow_x, flow_y));
	            ready.store(true);
                    data_points_mutex.unlock();

                    flow_x = flow_y = temp_flow_x = temp_flow_y = history = 0;
                }
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
