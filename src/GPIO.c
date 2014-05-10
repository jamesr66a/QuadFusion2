void initGPIO(int gpioNum){
  int fd;
  char buf[25];
  
  fd=open("/sys/class/gpio/export", O_WRONLY);
 
  sprintf(buf,"%d",gpioNum);

  write(fd,buf,strlen(buf));

  close(fd);

  sprintf(buf, "/sys/class/gpio/gpio%d/direction",gpio);
  fd=open(buf, O_WRONLY);
  write(fd, "in", 2);
  close(fd);
}

char readGPIO(int gpioNum){
  char retVal;
  char buf[25];

  sprintf(buf, "/sys/class/gpio/gpio%d/value",gpioNum);

  fd=open(buf, O_RDONLY);
  read(fd,&retVal,1);
  close(fd);

  return retVal;
}

