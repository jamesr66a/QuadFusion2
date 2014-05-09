/**********************************
 * encoder.h
 * 
 * Matthew Bailey
 * Using code from J-Works
 *
 * This file describes the J-Works 
 * encoder counter module and gives 
 * an interface to read data from 
 * the module over USB.
 **********************************/

#include <stdio.h>
#include <stdlib.h>

#include <stdio.h>
#include <sys/ioctl.h>
#include <ctype.h>
#include <limits.h>
#include <errno.h>
#include <stddef.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <unistd.h>
#include <stdlib.h>

#include </usr/include/libusb-1.0/libusb.h>

#define SERIALNUMBER_MAXLEN 	20
#define MAX_DEVICES 		8
#define MAX_CHANNELS		6
#define CTRL_IN			0xc2

// the following values are passed via the control value in the USB control pipe to the JSB524 for control
#define JSB524_FLASHLED_CMD	0xb1	// flash the module led
#define JSB524_VERSION_CMD	0xb4	// return the firmware version 
#define JSB524_NUM_CHANNELS_CMD	0xb2	// returns the number of channels 
#define JSB524_READ_COUNT_CMD	0xb6	// latches and returns count of all channels
#define JSB524_CLR_COUNT_CMD	0xb7	// clear the counters based on channel number, 7 clears all
#define JSB524_SETMODE_CMD	0xb8	// sets the counter mode
#define JSB524_SETMIN_CMD	0Xb9	// sets preset min value
#define JSB524_SETMAX_CMD	0xba	// sets preset max value
#define JSB524_SETDIR_CMD	0xbb	// set io direction bit, 1 = out
#define JSB524_SETOUT_CMD	0Xbc	// set output bit
#define JSB524_READIO_CMD	0xbd	// gets the io port data

static struct libusb_device_handle* jsb524devh[MAX_DEVICES];	// jsb524 devices attached up to max

struct encoderData {
  double data[MAX_DEVICES];
  int size;
};

class encoderCounter {
public:
  encoderCounter();
  ~encoderCounter();
  void readData(encoderData*);

private:
  int jsb524Init(void);
  void jsb524Close(void);
  int jsb524ReadCount(int, u_int16_t*, unsigned char*, int);
  int jsb524SetMode(int, unsigned char, unsigned char);
  int jsb524SetDir(int, unsigned char);
  int jsb524SetOut(int, unsigned char);
  int jsb524PreMin(int, unsigned char, u_int32_t);
  int jsb524PreMax(int, unsigned char, u_int32_t);
  int jsb524ClearCount(int, unsigned char);
  int jsb524NumberOfChannels(int);
  int jsb524ReadInput(int);
  int jsb524GetSerialNumber(int, unsigned char*, size_t);
  unsigned char jsb524Version(int);

  u_int16_t numJsbDevicesFound; // number of Devices found for this vendor id and product id	
  libusb_device** devs;						// all the usb devices attached to system
  encoderData bias;

  unsigned char caSerialNumber[SERIALNUMBER_MAXLEN];
  int iNumberOfChannels;

  int iNumDevices;
  int iRet;
  int iDeviceNum;
  int iLoop;
  u_int16_t pData[6];
  unsigned char pState[6];
  unsigned char ucChannelNumber;
};

encoderCounter::encoderCounter () {
  numJsbDevicesFound = 0; // number of Devices found for this vendor id and product id	
  iNumberOfChannels = 0;
  iNumDevices = 0;
  iRet = 0;
  iDeviceNum = 1;

  iNumDevices = jsb524Init();
  iNumberOfChannels = jsb524NumberOfChannels(iDeviceNum);

  // sets the counter mode 
  //  bit 0,1 define the count mode  0 = X1  1 = X2  2 = X4
  //  bit 2 when 1 causes the counter to cleared on index
  //  bit 3 is the index polarity
  jsb524SetMode(iDeviceNum,7,0x0);	// all channels, x4 index high true, clear on index
  // set preset values
  jsb524PreMin(iDeviceNum,7,0);
  jsb524PreMax(iDeviceNum,7,999);
  jsb524ClearCount(iDeviceNum, 7);

  for(int i = 0; i < iNumberOfChannels; ++i)
    bias.data[i] = 0;

  bias.size = 0;

//  for(int i = 0; i < iNumDevices; ++i)
//    jsb524ClearCount(iDeviceNum, i);

  readData(&bias);
}

encoderCounter::~encoderCounter () {
  jsb524Close();
}

void encoderCounter::readData (encoderData* data) {

  // NOTE: 'state' 
  //  bit 0,1: define the count mode  0 = X1  1 = X2  2 = X4
  //  bit 2: when 1, causes the counter to be cleared on index
  //  bit 3: is the index polarity
  iRet = jsb524ReadCount(iDeviceNum, pData, pState, iNumberOfChannels);
  data->size = iNumberOfChannels;

  for (iLoop = 0; iLoop < iNumberOfChannels; iLoop++) {
//    double rad_angle = (3.14159/500)*pData[iLoop];
//    double deg_angle = (0.36)*pData[iLoop];
    data->data[iLoop] = (M_PI/500)*pData[iLoop] - bias.data[iLoop];
//    data->data[iLoop] = (0.36)*pData[iLoop] - bias.data[iLoop];
  }
}

//***********************************************************************************************************
//  jsb524Init()
//
//	this functions finds out how many jsb524 device are connected. It gets the list of devices attached
//	uses the vendor id and product id to check on jsb524 devices. It then fills in a table for each device
//	For the jsb524 internal application 
//      most likely only one device will be used, so this code can be simplified
//
int encoderCounter::jsb524Init(void)
{
	int r,cnt;
	libusb_device *dev;
	struct libusb_device_descriptor desc;
	int i = 0;

	// vendor ID and product ID for jsb524 modules (from module firmware)
	u_int16_t vendorId  = 0x7c3;
	u_int16_t productId = 0x8524;

	r = libusb_init(NULL);		// init the libusb
	if (r < 0)
		return r;

	cnt = libusb_get_device_list(NULL, &devs);	// get a list of all usb devices connected
	if (cnt < 0)
		return (int) cnt;	// sorry no devices attached
	// check to see how many jsb524 devices are attached
	// and fill in table information for attached devices
	while ( ((dev = devs[i++]) != NULL) && (numJsbDevicesFound < MAX_DEVICES)) 
		{
		libusb_get_device_descriptor(dev, &desc);
		if (desc.idVendor == vendorId && desc.idProduct == productId)
			{
			libusb_open(dev,&jsb524devh[numJsbDevicesFound]);
			libusb_claim_interface(jsb524devh[numJsbDevicesFound], 0);
			numJsbDevicesFound++;
			}	
		}

	// tell caller how many jsb524 devices are there
	return numJsbDevicesFound;

}	// end of jsb524Init()
//***********************************************************************************************************

//***********************************************************************************************************
//
//	jsb524Close
//
//		does a clean close of the device 
//
void encoderCounter::jsb524Close(void)
{
	int i;

	for (i=0; i < numJsbDevicesFound;i++)
		libusb_close(jsb524devh[numJsbDevicesFound]);
	libusb_free_device_list(devs, 1);
	libusb_exit(NULL);

}    	// end of jsb524Close()
//***********************************************************************************************************

//***********************************************************************************************************
// jsb524ReadCount()
//
int encoderCounter::jsb524ReadCount(int iDeviceNum,u_int16_t pData[],unsigned char pState[],int iArraySize)
{
	unsigned char data[24];		// data returns by JSB524,
	int iRet;			// status return value
	int iLoop;
	int iIndex;

	// make sure we are in the range of open devices
	if (iDeviceNum > numJsbDevicesFound || iDeviceNum <= 0)
		return -EINVAL;
	iDeviceNum--;							// user sends index 1 base, but table is zero based
	// send command to read count 24 bytes
	iRet = libusb_control_transfer(jsb524devh[iDeviceNum], CTRL_IN,JSB524_READ_COUNT_CMD, 0, 0, data, 24, 0);
	if (iRet >= 0)
		{
		for (iLoop = 0; iLoop < iArraySize;iLoop++)
			{
			iIndex        = iLoop * 4;	
			// count is a 24 bit value	
			pData[iLoop]  = data[iIndex] + (data[iIndex + 1] << 8) + (data[iIndex + 2] << 16);
			// state 
			//  bit 0,1 define the count mode  0 = X1  1 = X2  2 = X4
			//  bit 2 when 1 causes the counter to cleared on index
			//  bit 3 is the index polarity
			pState[iLoop] = data[iIndex + 3];
			}
		}

	return iRet;
}	// end of jsb524ReadCount()
//***********************************************************************************************************

//***********************************************************************************************************
// jsb524SetMode()
//			 mode
//			  bit 0,1 define the count mode  0 = X1  1 = X2  2 = X4
//			  bit 2 when 1 causes the counter to cleared on index
//			  bit 3 is the index polarity
//
int encoderCounter::jsb524SetMode(int iDeviceNum, unsigned char ucChannelNumber, unsigned char ucMode)
{
	unsigned char data[2];		// data returns by JSB524,
	int iRet;			// status return value

	// make sure we are in the range of open devices
	if (iDeviceNum > numJsbDevicesFound || iDeviceNum <= 0)
		return -EINVAL;
	iDeviceNum--;							// user sends index 1 base, but table is zero based
	// send command to set mode
	iRet = libusb_control_transfer(jsb524devh[iDeviceNum], CTRL_IN,JSB524_SETMODE_CMD, ucMode, ucChannelNumber, data, 2, 0);
	return iRet;
}	// end of jsb524SetMode()
//***********************************************************************************************************

//***********************************************************************************************************
// jsb524SetDir()
int encoderCounter::jsb524SetDir(int iDeviceNum, unsigned char ucDir)
{
	unsigned char data[2];		// data returns by JSB524,
	int iRet;			// status return value

	// make sure we are in the range of open devices
	if (iDeviceNum > numJsbDevicesFound || iDeviceNum <= 0)
		return -EINVAL;
	iDeviceNum--;							// user sends index 1 base, but table is zero based
	// send command to set dir
	iRet = libusb_control_transfer(jsb524devh[iDeviceNum], CTRL_IN,JSB524_SETDIR_CMD, ucDir, 0, data, 2, 0);
	return iRet;
}	// end of jsb524SetDir()
//***********************************************************************************************************

//***********************************************************************************************************
// jsb524SetOut()
int encoderCounter::jsb524SetOut(int iDeviceNum, unsigned char ucOut)
{
	unsigned char data[2];		// data returns by JSB524,
	int iRet;			// status return value

	// make sure we are in the range of open devices
	if (iDeviceNum > numJsbDevicesFound || iDeviceNum <= 0)
		return -EINVAL;
	iDeviceNum--;							// user sends index 1 base, but table is zero based
	// send command to set out
	iRet = libusb_control_transfer(jsb524devh[iDeviceNum], CTRL_IN,JSB524_SETOUT_CMD, ucOut, 0, data, 2, 0);
	return iRet;
}	// end of jsb524SetOut()
//***********************************************************************************************************

//***********************************************************************************************************
// jsb524PreMin()
int encoderCounter::jsb524PreMin(int iDeviceNum, unsigned char ucChannelNumber,u_int32_t uiPreValue )
{
	unsigned char data[2];		// data returns by JSB524,
	int iRet;			// status return value
	u_int16_t uiIndex;
	u_int16_t uiValue;

	uiIndex = ucChannelNumber + ( (uiPreValue & 0xff0000) >> 8);
	uiValue = (uiPreValue & 0xffff);
	// make sure we are in the range of open devices
	if (iDeviceNum > numJsbDevicesFound || iDeviceNum <= 0)
		return -EINVAL;
	iDeviceNum--;							// user sends index 1 base, but table is zero based
	// send command to set mode
	iRet = libusb_control_transfer(jsb524devh[iDeviceNum], CTRL_IN,JSB524_SETMIN_CMD, uiValue,uiIndex, data, 2, 0);
	return iRet;
}	// end of jsb524PreMin()
//***********************************************************************************************************

//***********************************************************************************************************
// jsb524PreMax()
int encoderCounter::jsb524PreMax(int iDeviceNum, unsigned char ucChannelNumber,u_int32_t uiPreValue )
{
	unsigned char data[2];		// data returns by JSB524,
	int iRet;			// status return value
	u_int16_t uiIndex;
	u_int16_t uiValue;

	uiIndex = ucChannelNumber + ( (uiPreValue & 0xff0000) >> 8);
	uiValue = (uiPreValue & 0xffff);

	// make sure we are in the range of open devices
	if (iDeviceNum > numJsbDevicesFound || iDeviceNum <= 0)
		return -EINVAL;
	iDeviceNum--;							// user sends index 1 base, but table is zero based
	// send command to set mode
	iRet = libusb_control_transfer(jsb524devh[iDeviceNum], CTRL_IN,JSB524_SETMAX_CMD, uiValue,uiIndex, data, 2, 0);
	return iRet;
}	// end of jsb524PreMax()
//***********************************************************************************************************

//***********************************************************************************************************
// jsb524ClearCount()
int encoderCounter::jsb524ClearCount(int iDeviceNum, unsigned char ucChannelNumber)
{
	unsigned char data[2];		// data returns by JSB524,
	int iRet;			// status return value

	// make sure we are in the range of open devices
	if (iDeviceNum > numJsbDevicesFound || iDeviceNum <= 0)
		return -EINVAL;
	iDeviceNum--;							// user sends index 1 base, but table is zero based
	iRet = libusb_control_transfer(jsb524devh[iDeviceNum], CTRL_IN,JSB524_CLR_COUNT_CMD, 0, ucChannelNumber, data, 2, 0);
	return iRet;
}	// end of JSb524ClearCount()
//***********************************************************************************************************

//***********************************************************************************************************
// jsb524NumberOfChannels()
//
int encoderCounter::jsb524NumberOfChannels(int iDeviceNum)
{
	unsigned char data[6];		// data returns by JSB524,
	int iRet;			// status return value

	// make sure we are in the range of open devices
	if (iDeviceNum > numJsbDevicesFound || iDeviceNum <= 0)
		return -EINVAL;
	iDeviceNum--;							// user sends index 1 base, but table is zero based
	iRet = libusb_control_transfer(jsb524devh[iDeviceNum], CTRL_IN,JSB524_NUM_CHANNELS_CMD, 0, 0, data, 6 ,0);
	if (iRet < 0)
		return iRet;
	else
		return data[0];


}	// end of jsb524NumberOfChannels()
//***********************************************************************************************************

//***********************************************************************************************************
// jsb524ReadInput()
//
int encoderCounter::jsb524ReadInput(int iDeviceNum)
{
	unsigned char data[2];		// data returns by JSB524,
	int iRet;			// status return value

	// make sure we are in the range of open devices
	if (iDeviceNum > numJsbDevicesFound || iDeviceNum <= 0)
		return -EINVAL;
	iDeviceNum--;							// user sends index 1 base, but table is zero based
	// send command to get io data
	iRet = libusb_control_transfer(jsb524devh[iDeviceNum], CTRL_IN,JSB524_READIO_CMD, 0, 0, data, 6, 0);
	if (iRet < 0)
		return iRet;
	else
		return data[0];


}	// end of jsb524ReadInput()
//***********************************************************************************************************

//***********************************************************************************************************
// jsb524GetSerialNumber()
//
int encoderCounter::jsb524GetSerialNumber(int iDeviceNum, unsigned char* data,size_t length)
{
	int iRet = 0;												// status return value
	// make sure we are in the range of open devices
	if (iDeviceNum > numJsbDevicesFound || iDeviceNum <= 0)
		return -EINVAL;
	if (length < SERIALNUMBER_MAXLEN)
		return -EINVAL;

	iDeviceNum--;							// user sends index 1 base, but table is zero based
	// send command
	iRet = libusb_get_string_descriptor_ascii(jsb524devh[iDeviceNum],5,data,length);
	return iRet;		// return error code
}	// end of jsb524GetSerialNumber()
//***********************************************************************************************************

//***********************************************************************************************************
// jsb524Version()
//
unsigned char encoderCounter::jsb524Version(int iDeviceNum)
{
	unsigned char data[2];		// data returns by JSB524,
	int iRet = 0;

	if (iDeviceNum > numJsbDevicesFound || iDeviceNum <= 0)
		return -EINVAL;

	iDeviceNum--;							// user sends index 1 base, but table is zero based
	// send command
	iRet = libusb_control_transfer(jsb524devh[iDeviceNum], CTRL_IN,JSB524_VERSION_CMD, 0, 0, data, 2, 0);
	return data[0];		// return error code
}	// end of jsb524Version()
//***********************************************************************************************************

/*
int main(int argc, char *argv[]) {
    FILE* config = fopen("config.dat", "r" );
    if(config == NULL)
    {
       printf("Can't open configuration file");
       return 1;
    }

    char identifier[256];
    int  value;

    while(!feof(config))
    {
        fscanf(config, "$%s %d\n", identifier, &value);
        if(strcmp(identifier, "COLLECTION_MODE") == 0)
            collection_mode = value;
    }

}
*/

/* EOF */

