# MLX90640_Stream

Stream image data from the MLX90640 thermal camera. The stream is accessible with e.g. VLC by using Media -> "Open Network Stream..." and following URL:

`http://<esp32-ip>/stream` 

## Sources
Based on repo https://github.com/arkhipenko/esp32-mjpeg-multiclient-espcam-drivers with the subproject: https://github.com/arkhipenko/esp32-mjpeg-multiclient-espcam-drivers/tree/master/esp32-cam-rtos

* https://github.com/adafruit/Adafruit_MLX90640.git
* https://github.com/blackcj/esp32-thermal-camera.git
* https://github.com/melexis/mlx90640-library.git
* https://github.com/sparkfun/SparkFun_MLX90640_Arduino_Example.git

## TODO
* investigate occassional reset after client disconnect
