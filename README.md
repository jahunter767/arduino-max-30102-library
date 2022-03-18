# arduino-max-30102-library

This library was written with the intension of simplifying the library provided
by SparkFun [here](https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library)
as well as separate the i2c communication logic from the code to interact with
the module.

## TODO

1. explore reducing the memory requirements by
   1. finding an alternative to storing all those register masks
   2. using a bitfield to store a module's configuration and having logic to
      intelligently sends the relevant commands to apply the configuration
2. debug the algorithm to compute the heart rate

## Useful resources

[Arduino Heart Rate Monitor Using MAX30102 and Pulse Oximetry](https://makersportal.com/blog/2019/6/24/arduino-heart-rate-monitor-using-max30102-and-pulse-oximetry)

[Calibration-Free Pulse Oximetry Based on Two Wavelengths in the Infrared — A Preliminary Study](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4029673/)

[Pulse Oximeter Fundamentals and Design](https://www.nxp.com/docs/en/application-note/AN4327.pdf)
