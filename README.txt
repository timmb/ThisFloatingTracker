This Floating Tracker
=====================

Overview
------------

_This Floating World_ is an interactive audiovisual dance piece created by Tim Murray-Browne and Jan Lee and first performed in London in 2015.  
http://thisfloatingworld.com

This Floating Tracker (TFT) is the framework and movement analysis software used in the production _This Floating World_ . It analyses the movement of a dancer using a Kinect v2 and provides a basis to run generative graphic sketches, signal control data via OSC and MIDI, and receive OSC control from TouchOSC running on a tablet.

TFT includes all the software of the show except for the graphics content, which I'm hoping to be able to include at a later date.


Usage
-----
TFT is built against Cinder 0.8.6, available from https://github.com/cinder/Cinder/tree/v0.8.6 using Visual Studio 2013. The `Cinder` folder should be placed in the same directory as the ThisFloatingTracker folder.

It requires a Kinect v2 to run with the Microsoft Kinect SDK installed, and therefore only runs on Windows 8+. TFT relies on the Kinect driver to extract the dancer's silhouette from the depth stream. Everything else is done in the software included here, so in theory it could be ported to OSX using LibFreeNect2 and a background subtraction algorithm.

In its current form the application allows you to visualise the different tracking methods in place. The main ones of interest that we used in the show are:

- UserStats, including `qom` (quantity-of-motion) used in the opening section and other measurements
- ControlPoints, which draw the strokes in the middle section of the piece. (Select mDrawControlPoints2d to see the points we used. 3D points had issues that I didn't have time to fix but I've left the code in there.)
- OpticalFlow, which we used to move the plants from the dancer's motion in the middle and final section of the piece.

You can create your own graphics sketches by subclassing the Sketch class and creating an instance in ThisFloatingTrackerApp::setup() as indicated in the comments.

There are a number of coordinate spaces in play:
- Kinect space, the matrix of the input depth image, e.g. (0,0) to (320, 240). Kinect images arrive in these coordinates.
- Render space, a resolution-independent coordinate space from (-r, r) to (-1, 1) where r is defined by the aspect ratio, typically around 1.4. This is used everywhere for 2D graphics and rendering (including control point positions, etc). This coordinate system is set using the setMatricesRender function. It is essentially the same as normalized coordinates in OpenGL but corrected for the aspect ratio of the render target.
- Window space, the pixels of the application window.

The render resolution is defined by gRenderTargetSize but ordinarily you wouldn't refer to this. This lets us change the render resolution of the work without changing the location of things in 2D space.

Input data is then accessed through the global gInput pointer defined in Common.h. e.g. 

    // Access to all of these is thread safe. It's faster to access once and keep your 
    // copy rather than making repeated calls.
    gInput->flow.getFlow(); // returns the current optical flow as a cv::Ma
    gInput->userStats.getUserStats(); // returns the latest UserStats in an object
    gInput->tracker.getControlPoints(); // returns a vector of ControlPoint instances.

There's a fair amount of values in a ControlPoint, not all of which are bug-free. In This Floating World we used `pos2d`, `vel2d`, and `dynamicControl`. I have spotted bugs present in this tracking code and there are many improvements that could be made to it. As in this initial release, this is the tracking we use in the show.

The TouchOSC layout is included in `other` directory. See ThisFloatingTracker::setup() for samples of how to quickly map touchOSC controls to variables in the application.

Licence
-------

This code is released under the MIT licence to save you hassle. We request that you acknowledge the use of this project in any code you use, ideally with a link to http://thisfloatingworld.com.

(c) 2015 Tim Murray-Browne and Jan Lee
