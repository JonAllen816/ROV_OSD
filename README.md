# ROV_OSD

Work in Progress...

https://discuss.ardupilot.org/t/please-help-trying-to-use-mavlink-for-minimosd-via-arduino-without-apm-pixhawk/37373

I have built an ROV largely based on http://techmonkeybusiness.com/rov-control-sketches-fourth-edition.html 2 - It uses 2 Arduino nanos to communicate via EasyTransfer.h and currently it receives sensor data and prints it on a LCD.

I am now trying to use the minimosd to show my sensor data on my FPV goggles - I have searched a ton of forums and tried several complicated examples based on ArduPilots from github but I am really struggling to decode Mavlinks protocols to display simple data from my sensors [ IMU (MPU9250), Temperature/Pressure sensor (MS5803), RTC (DS3231) and external PSI/Depth sensor (A6), voltage (A7) ].
