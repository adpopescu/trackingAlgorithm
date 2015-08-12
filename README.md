-------------------------------------------------------------------------------
-  RealPDT (modified) - Real-time People Detection and Tracking system
-------------------------------------------------------------------------------

This is a modified version of the real-time RGB-D based people detection and tracking system,
which can get its input from kinect based cameras or recorded sequences 
of depth cameras.

This distribution removes support for stereo cameras and adds support for loading sequences
recorded with freenect (fakenect).

The original system is presented in

Real-Time RGB-D based People Detection and Tracking for Mobile Robots and Head-Worn Cameras
O. Hosseini Jafari, D. Mitzel, B. Leibe
IEEE International Conference on Robotics and Automation (ICRA'14), 2014


-----------------------------------------------
In this implementation, the following libraries are used (See their files for their corresponding licenses):

	* FOVIS - a visual odometry library (GPLv2 Licensed)
	* Openni x64 v2.2 - we used this library to get the data from kinect based cameras
	* CImg - a library for loading and saving images
	* KConnectedComponentLabeler - It is used for finding connected labels

-----------------------------------
DESCRIPTION
-----------------------------------

RealPDT is a fully integrated people detection and tracking system which can be
run on pre-recorded sequences or can be run with a kinect based camera connected 
to your device (we tested it with Microsoft Kinect and Asus Xtion). The parameters 
must be passed to the application as a config file (*.inp). There are two template
config files in "./RealPDT/bin/" directory (config_Asus.inp for using the online 
camera, and config_sunnyday.inp for using the pre-recorded sequences from stereo 
cameras).

The following components are distributed:

	* RealPDT			- Real-time Peaople Detection and Tracking system
	* Depth upper-body Template	- A normalized depth upper-body template for using in
					  upper-body detector (upper_temp_n.txt)
	* cudaHOG library		- GPU-based HOG detector for far-range detection
	* cudaHOG model	  		- this is a model for pedestrian detection trained
					  on the INRIAPerson dataset.
	* Openni linux x64 v2.2		- A binary version of Openni v2.2 which works on our systems,
					  but you may need to download the source code and build it on 
					  your system.

DISCLAIMER:
The cudaHOG code is based on an unmaintained branch of groundHOG by Patrick Sudowe, Computer Vision Group, 
RWTH Aachen University. If you want to use cudaHOG for other projects, please download the official 
maintained release from http://www.vision.rwth-aachen.de/software.
 
-----------------------------------
COPYING  - LICENCE
-----------------------------------

If you use this software we ask you to cite our paper:
BibTex:

@InProceedings{HosseiniJafari14ICRA,
  author =       {O. Hosseini Jafari, D. Mitzel and B. Leibe},
  title =        {{Real-Time RGB-D based People Detection and Tracking for Mobile Robots and Head-Worn Cameras}},
  booktitle =    ICRA,
  year =         {2014},
}

The software is released under the BSD licence. A copy of the licence can be found
in the COPYING.txt that is included with this software.

THIS CODE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE. Use at your own risk.

-----------------------------------
Dependencies
-----------------------------------

* NVIDIA CUDA enabled GPU in your system (Optional - for using far range groundHOG detector)
* CUDA installed
* cmake 	- build tool
* boost
* eigen3
* ImageMagick++
* Openni v2.2 installed and worked (we used Openni x64 v2.2, you can download the binary version 
	      and instal it. In some cases the binary version doesn't work, so you can download 
	      the source code and compile it. In both cases, please put the include files in 
	      ./OpenNI-Linux-x64-2.2/Include/ and the library files in ./OpenNI-Linux-x64-2.2/Redist.)

-----------------------------------
BUILD
-----------------------------------

Run the following commands in the project top-level directory:

$ mkdir build
$ cd build
$ cmake ..
$ make

This should build the main project, RealPDT, and the library cudaHOG. If you are missing any 
of the above dependencies you have to install them.

-----------------------------------
USAGE
-----------------------------------
You need to export the cudaHOG directory using following command:
	
	export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$DIR/../../cudaHOG/

Example: (in ./RealPDT/bin/ directory)

	./RealPDT -c config_Asus.inp

The command line option:

-c	<config_file_address>

-----------------------------------
SUPPORT
-----------------------------------

This software is developed and tested on Linux (Ubuntu 14.04 64Bit).

For this modified distribution you may contact
David Tenty <dtenty@scs.ryerson.ca>
Adrian Popescu <dapopesc@scs.ryerson.ca>

For the original authors you may contact
Omid Hosseini Jafari <omid.hosseini.jafari@rwth-aachen.de>
Stefan Breuers <breuers@vision.rwth-aachen.de>

If you may find any bugs or want to suggest improvements you are
most welcome to contact us. If you want to submit any patches, we
prefer git patch format.

-----------------------------------
ORIGINAL AUTHORS
-----------------------------------
Omid Hosseini Jafari <omid.hosseini.jafari@rwth-aachen.de>
Stefan Breuers <breuers@vision.rwth-aachen.de>

Original source can be downloaded from www.vision.rwth-aachen.de/software
