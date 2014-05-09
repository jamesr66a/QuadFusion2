#ifndef _ECON_H
#define _ECON_H

/*****************************************************
 * econ.h
 *
 * Matthew Bailey, Chris Wakeley
 * ASCL: NEEC Navy Robot Project
 * 2013-10-14
 *
 * This file describes an econ camera device that
 * is used to retrieve an image frame from the camera.
 * This code is based on the code provided by econ
 * camera systems.
 *
 *****************************************************/

#include <iostream>
#include <iomanip>
#include <sys/ioctl.h>
#include <stdio.h>
#include <fstream>

#include <stdlib.h>
#include <stddef.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <string>
#include <sstream>

#include <chrono>
#include <iostream>

#include <linux/videodev2.h>
#include "opencv2/core/core.hpp"

#define NOTDEBUG

const int defaultWidth = 1600;
const int defaultHeight = 1200;
const int defaultDevice = 2;
const char UYVY = 0x00;
const char YUYV = 0x01;

struct camera {
	uint8_t* buffer;
	uint8_t* data;
	cv::Mat* img;
	struct v4l2_format fmt;
	struct v4l2_buffer buf;
	struct { int w; int h; int frameSize; } res;
	int device;
};

class econ {
public:
	econ(int videoDeviceNum = defaultDevice, int width = defaultWidth, int height = defaultHeight);
	~econ();
	int readImg(cv::Mat&);
	int readCV(cv::Mat&);

private:
	camera* cam;
	int readRGB();
	int readRaw();
	int convertBuff2RGB(char mode = UYVY);
	int convertBuff2CV(cv::Mat*);
	int writeRaw();
	int writeRGB();
	int getFormat();
};

/* econ
   constructor: create an econs camera object
 
   arguments:#include "opencv2/core/core.hpp"
     videoDeviceNum: the video device associated with the camera
     width         : the width of a frame
     height        : the height of a frame
   returns
     none
*/
econ::econ (int videoDeviceNum, int width, int height) {

	int frameSize = width * height;

	// INITIALIZE CAMERA
	cam = new camera;
	if(!cam) {
		std::cerr << "MEMEORY COULD NOT BE ALLOCATED FOR CAMERA DEVICE" << std::endl;
		exit(-1);
	} 
	
	std::stringstream deviceName;
	deviceName << "/dev/video" << videoDeviceNum;
	cam->device = open(deviceName.str().c_str(), O_RDWR | O_NONBLOCK, 0);
 //       fcnl(cam->device, FL_SET, O_NONBLOCK);
	if (cam->device < 0) {
		std::cerr << "COULD NOT OPEN VIDEO DEVICE " << deviceName << std::endl;
		exit(-1);
	}
		
	cam->res.w = width;
	cam->res.h = height;
	cam->res.frameSize = frameSize;
/*{* OLD BUFFER SETUP* /
	cam->data = new uint8_t[frameSize*3];

	if (!cam->data) {
		std::cerr << "COULD NOT ALLOCATE CAMERA DATA" << std::endl;
		exit(-1);
	}
/**/
	cam->buffer = new uint8_t[frameSize*2];
	if(!cam->buffer) {
		std::cerr << "COULD NOT ALLOCATE RAW DATA BUFFER" << std::endl;
		exit(-1);
	}

	cam->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(ioctl(cam->device, VIDIOC_G_FMT, &cam->fmt) < 0) {
		std::cerr << "COULD NOT GET DEVICE FORMAT" << std::endl;
		exit(-1);
	}

	cam->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	cam->fmt.fmt.pix.width = cam->res.w;
	cam->fmt.fmt.pix.height = cam->res.h;
  cam->fmt.fmt.pix.sizeimage = cam->res.frameSize;
	cam->fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
	cam->fmt.fmt.pix.field = V4L2_FIELD_NONE;

	if (ioctl(cam->device, VIDIOC_S_FMT, &cam->fmt) < 0) {
		std::cerr << "COULD NOT SET DEVICE FORMAT" << std::endl;	
		exit(-1);
	}

	#ifdef DEBUG
	std::cout << "CAMERA SUCCESSFULLY SET UP" << std::endl;
	#endif
}

/* ~econ
   destructor: destroys an econ camera object
 
   arguments:
     none
   returns:
     none
*/
econ::~econ () {
//	delete [] cam->data;
	delete [] cam->buffer;
	delete cam;
}

int econ::readImg (cv::Mat& img) {
//!	this->readRGB();  

	this->readRaw();
	this->convertBuff2CV(&img);

	#ifdef DEBUG
	this->getFormat();
	this->writeRaw();
	this->writeRGB();

//	cv::imwrite("./output/testCam.jpg", *cam->img);
	cv::imwrite("./output/testImg.jpg", img);
	#endif

  return 0;
}

int econ::readRGB () {
	int returnVal = readRaw();
	returnVal += convertBuff2CV(cam->img);
	return returnVal;
}

int econ::readRaw () {

	#ifdef DEBUG
	std::cout << "fmt.fmt.pix.sizeimage = " << cam->fmt.fmt.pix.sizeimage << std::endl;
	#endif
	
	auto start = std::chrono::high_resolution_clock::now();
	int readBits = read(cam->device, (void*)cam->buffer, cam->fmt.fmt.pix.sizeimage);
	auto end = std::chrono::high_resolution_clock::now();
	auto elapsed = end - start;

	std::cout << elapsed.count() << std::endl;	

	if(readBits != cam->fmt.fmt.pix.sizeimage) {
		std::cerr << "Read bits != image size" << std::endl;
		return -1;
	}

#ifdef DEBUG
        printf("Succesfully read raw image\n");
#endif

	return 0;
}

int econ::convertBuff2RGB (char mode) {

	int height = cam->res.h;
	int width  = cam->res.w;
	int count  = 0;
	int temp;
	float u_val, v_val, y1_val, y2_val;

	for (int i=0; i < height; i++) {
		for (int j = 0; j < width; j+=2) {
			u_val  = (float)cam->buffer[count + ((mode == UYVY)? 0 : 1)] - 128;	
			y1_val = (float)cam->buffer[count + ((mode == UYVY)? 1 : 0)];
			v_val  = (float)cam->buffer[count + ((mode == UYVY)? 2 : 3)] - 128;
			y2_val = (float)cam->buffer[count + ((mode == UYVY)? 3 : 2)];

			temp = (int)(y1_val + (1.770 * u_val));
			cam->data[(((height-1)-i) * width * 3) + j*3 +0] = (temp > 255) ? 255 : ((temp < 0) ? 0 :(char)temp);

			temp = (int)(y1_val - (0.344 * u_val) - (0.714 * v_val));
			cam->data[(((height-1)-i) * width * 3) + j*3 +1] = (temp > 255) ? 255 : ((temp < 0) ? 0 :(char)temp);
				
			temp = (int)(y1_val + (1.403 * v_val));
			cam->data[(((height-1)-i) * width * 3) + j*3 +2] = (temp > 255) ? 255 : ((temp < 0) ? 0 :(char)temp);
	
			temp = (int)(y2_val + (1.770 * u_val));
			cam->data[(((height-1)-i) * width * 3) + j*3 +3] = (temp > 255) ? 255 : ((temp < 0) ? 0 :(char)temp);
	
			temp = (int)(y2_val - (0.344 * u_val) - (0.714 * v_val));
			cam->data[(((height-1)-i) * width * 3) + j*3 +4] = (temp > 255) ? 255 : ((temp < 0) ? 0 :(char)temp);
				
			temp = (int)(y2_val + (1.403 * v_val));
			cam->data[(((height-1)-i) * width * 3) + j*3 +5] = (temp > 255) ? 255 : ((temp < 0) ? 0 :(char)temp);

			count += 4;

		}
	}
	return 0;
}

int econ::convertBuff2CV (cv::Mat* img) {

#ifdef DEBUG
	std::cout << "BEGINING CONVERSION" << std::endl;
#endif

    int width = cam->fmt.fmt.pix.width;
    int height = cam->fmt.fmt.pix.height;
    char gb, rg, r, g, b;

    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {

            gb = cam->buffer[(i*width*2)+j*2+0];
            rg = cam->buffer[(i*width*2)+j*2+1];
            r = (rg & 0xF8);
            g = (((rg & 0x7) << 3) | ((gb & 0xE0) >> 5)) << 2;
            b = ((gb & 0x1F) << 3);

//          cam->data[(((height-1)-i)*width*3)+j*3+0] = 0xFF & b;
//          cam->data[(((height-1)-i)*width*3)+j*3+1] = 0xFF & g;
//          cam->data[(((height-1)-i)*width*3)+j*3+2] = 0xFF & r;

            int i0 = img->step[0];
            int j0 = img->step[1];

			img->data[i0*i + j0*j + 0] = 0xFF & b;
            img->data[i0*i + j0*j + 1] = 0xFF & g;
            img->data[i0*i + j0*j + 2] = 0xFF & r;
        }
    }

#ifdef DEBUG
	std::cout << "CONVERSION COMPLETE" << std::endl;
#endif

    return 0;
}


int econ::writeRaw () {
	const char* filename = "./output/rawImage.uyvy";
	
//!	std::ofstream outfile(filename, std::ios::out | std::ios::binary);

//	std::ofstream outfile(filename, std::ios::out);
//	outfile << "P6" << std::endl << "# ECON RAW DATA: Matthew Bailey" << std::endl << cam->res.w << " " << cam->res.h << std::endl << "255" << std::endl;
//	outfile.close();

//	outfile.open(filename, std::ios::out | std::ios::binary | std::ios::app);
//!	outfile.write((char*)cam->buffer, cam->res.frameSize*2);
//!	outfile.close();


	int rawFile = open(filename, O_WRONLY | O_CREAT | O_SYNC);
	write(rawFile, cam->buffer, cam->res.frameSize*2); 
	close(rawFile);

	return 0;
}

int econ::writeRGB () {
	const char* fileName = "./output/rgbImage.ppm";
	std::ofstream outfile(fileName, std::ios::out);
	outfile << "P6" << std::endl << "# ECON camera Image: Matthew Bailey" << std::endl << cam->res.w << " " << cam->res.h << std::endl << "255" << std::endl;
	outfile.close();
	
	outfile.open(fileName, std::ios::out | std::ios::app | std::ios::binary);
	outfile.write((char*)cam->data, cam->res.frameSize*3);
	outfile.close();
//	write(rgbFile, cam->data, cam->res.frameSize*3);
//	close(rgbFile);
//	return rgbFile;

	return 0;
}

int econ::getFormat () {


	switch(cam->fmt.fmt.pix.pixelformat) {
		case V4L2_PIX_FMT_RGB565: {
			printf("V4L2_PIX_FMT_RGB565\n");
		}break;

		case V4L2_PIX_FMT_UYVY: {
			printf("V4L2_PIX_FMT_UYVY\n");
		}break;

		case V4L2_PIX_FMT_YUYV: {
			printf("V4L2_PIX_FMT_YUYV\n");	
		}break;

		case V4L2_PIX_FMT_SBGGR8: {
			printf("V4L2_PIX_FMT_SBGGR8\n"); 
		}break;
	}

	return 0;
}

#endif

