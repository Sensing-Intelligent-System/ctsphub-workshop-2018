# installLibrealsenseTX2
JetsonHacks Install librealsense for Intel RealSense cameras on Jetson TX2 Development Kit

This is tested for version L4T 28.2 (JetPack 3.2)

To install the librealsense library:

$ ./installLibrealsense.sh

Which will install needed dependencies, and then the librealsense repository. The install script will also set udev rules so that the camera may be used in user space.

Once the setup is complete, the library will be built and installed.

UVC modules are now need to patched

Run patch-realsense-ubuntu-xenial-jetson-tx2.sh
Upon kernel configuration. select General Setup, and append 'JetsonTX2' (or your favourite name) to Local Version
In Edit/Find search CH341 and select USB Winchiphead.
Save and close confuguration.

Note that earlier versions of these scripts are in Tags.
Note that currently SR300 is only tested on realsense SDK2.

