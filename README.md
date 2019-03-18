# espcopter
This repo contains the firmware of espcopter.


## How to burn bin file
```
$ /usr/local/bin/esptool.py -vv -cd nodemcu -cb 115200 -cp /dev/ttyUSB0 -ca 0x00000 -cf remXY9250.ino.nodemcu.bin
```
