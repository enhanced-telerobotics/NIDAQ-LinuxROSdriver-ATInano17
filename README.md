# ROS/Linux Interface for niDAQ with ATI Nano17


TI doesn't have good support for Ubuntu Linux and thus we use the comedi library to create an interface for the niDAQ that reads the outputs from a ATI Nano17 force sensor and publishes the topic to ROS.

## Requires
You will need to download, compile, and install the [comedi](https://www.comedi.org/) library to use this package. 

## Additional Instructions/Comments
Might need to create a new package and configure it compile settings before the package can be used. The original package name was daq_fs 

Please place the ATI .cal file in the src directory 
