#include "GPIO.h"
/*
Direction:
false=out
true=in
*/

#include <unistd.h>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <cstring>

void initGPIO(int gpioNum, bool direction){ 
  int fd;
  char buf[25];
  
  fd=open("/sys/class/gpio/export", O_WRONLY);
  if(fd==-1) printf("Error");  
 
  else{ 
    sprintf(buf,"%d",gpioNum);

    write(fd,buf,strlen(buf));

    close(fd);

    sprintf(buf, "/sys/class/gpio/gpio%d/direction",gpioNum);
    fd=open(buf, O_WRONLY);
    if(direction) write(fd, "in", 2);
    if(!direction) write(fd, "out", 3);
    close(fd);

  }
}

char readGPIO(int gpioNum){
  char retVal;
  char buf[25];

  sprintf(buf, "/sys/class/gpio/gpio%d/value",gpioNum);

  int fd=open(buf, O_RDONLY);
  if(fd==-1) printf("Error");
  
  else{
    read(fd,&retVal,1);
    close(fd);
  }
  return retVal;
}
/*
gpioStatus:
true: write 1
flase: write 0
*/
void writeGPIO(int gpioNum, bool gpioStatus){
  char buf[25];
  
  sprintf(buf, "/sys/class/gpio/gpio%d/value",gpioNum);

  int fd=open(buf, O_WRONLY);
  if(fd==-1) printf("Error");

  else{
    if(gpioStatus) write(fd, "1", 1);
    if(!gpioStatus) write(fd, "0", 1);
    close(fd);
  }
}
