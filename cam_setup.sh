#!/bin/sh 

#Set the pipes 
#media-ctl -r -l '"mt9v032 3-005c":0->"OMAP3 ISP CCDC":0[1], "OMAP3 ISP CCDC":2->"OMAP3 ISP preview":0[1], "OMAP3 ISP preview":1->"OMAP3 ISP resizer":0[1], "OMAP3 ISP resizer":1->"OMAP3 ISP resizer output":0[1]'

#Set the formats 
#media-ctl -f '"mt9v032 3-005c":0[SGRBG10 752x480], "OMAP3 ISP CCDC":2[SGRBG10 752x480], "OMAP3 ISP preview":1[UYVY 752x480], "OMAP3 ISP resizer":1[UYVY 752x480]'

#Live coverage 
#LD_PRELOAD=/usr/lib/libv4l/v4l2convert.so mplayer tv:// -tv driver=v4l2:device=/dev/video6:outfmt=uyvy:width=752:height=480 -vf screenshot -vo jpeg -frames 5

modprobe v4l2-driver; \
mknod /dev/video2 c 81 0; \
chmod 777 /dev/video2
