# simple-kinect-blob-tracker

A simple, no fuss, C++ blob tracking software to use with Microsoft Kinect. Outputs data over OSC. Build using CinderLib, libfreenect, OpenCV.

It provides simple blob tracking, the possibility to record and play QuickTime movie files and tracking parameters are saved between sessions.

Tested with the 1st version of the Kinect on MacBook Pro running Yosemite.

### CinderBlocks dependencies
Cinder-Kinect: https://github.com/cinder/Cinder-Kinect
Cinder-OpenCV: https://github.com/cinder/Cinder-OpenCV
Cinder-Config: https://github.com/dawidgorny/Cinder-Config

### Usage instructions
When you startup the program it will check for a connected Kinect sensor. If one is found you will see the filtered depth data feed in the right half of the window, while the left part will show you the blob detection result in real time.

If no Kinect sensor is found when the software starts, you will be prompted to select a QuickTime movie file to load instead. This features allows you to calibrate the filters and tracking parameters from pre-recorded footage.

To record a movie from the Kinect feed, press the SPACE key on your keyboard, you will then be prompted to select a location on your hard drive where the movie file will be saved. To stop recording press the SPACE key again.

The tracker's configuration are automatically saved when the program shuts down.

### OSC
Since this tool was made for the purpose of exhibiting Mimodek in Maribor, Slovenia, where we only needed to track the presence of people in the space and there distance from the projection screen, only the number of blobs detected and the normalised [0..1] Y coordinate of the object closest to the top of the frame are sent through OSC. When I have more time to refine this software I'll make it output blobs centroids, etc...

/mimodek/blobs/
=> (int) blob count
=> (float) normalised [0..1] Y coordinate of the object closest to the top of the frame


