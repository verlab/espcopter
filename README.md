# espcopter
This repo contains the firmware of espcopter.


## How to burn bin file
´´´
sh
$ home/rezeck/.arduino15/packages/esp8266/tools/esptool/0.4.13/esptool -vv -cd nodemcu -cb 115200 -cp /dev/ttyUSB0 -ca 0x00000 -cf remXY9250.ino.nodemcu.bin

´´´
