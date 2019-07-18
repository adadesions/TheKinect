/*
-- Georgia Tech 2016 Spring
--
-- This is a sample code to show how to use the libfreenet2 with OpenCV
--
-- The code will streams RGB, IR and Depth images from an Kinect sensor.
-- To use multiple Kinect sensor, simply initial other "listener" and "frames"
-- This code refered from sample code provided from libfreenet2: Protonect.cpp
-- https://github.com/OpenKinect/libfreenect2
-- and another discussion from: http://answers.opencv.org/question/76468/opencvkinect-onekinect-for-windows-v2linuxlibfreenect2/
-- Contact: Chih-Yao Ma at <cyma@gatech.edu>
*/

//! [headers]
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>
#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
//! [headers]

using namespace std;
using namespace cv;

bool protonect_shutdown = false; // Whether the running application should shut down.

void sigint_handler(int s)
{
    protonect_shutdown = true;
}

int main()
{
    std::cout << "Streaming from Kinect One sensor!" << std::endl;

    //! [context]
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    //! [context]

    //! [discovery]
    if (freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }

    string serial = freenect2.getDefaultDeviceSerialNumber();

    std::cout << "SERIAL: " << serial << std::endl;

    if (pipeline)
    {
        //! [open]
        dev = freenect2.openDevice(serial, pipeline);
        //! [open]
    }
    else
    {
        dev = freenect2.openDevice(serial);
    }

    if (dev == 0)
    {
        std::cout << "failure opening device!" << std::endl;
        return -1;
    }

    signal(SIGINT, sigint_handler);
    protonect_shutdown = false;

    //! [listeners]
    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color |
                                                libfreenect2::Frame::Depth |
                                                libfreenect2::Frame::Ir);
    libfreenect2::FrameMap frames;

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    //! [listeners]

    //! [start]
    dev->start();

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
    //! [start]

    //! [registration setup]
    libfreenect2::Registration *registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);
    //! [registration setup]

    Mat rgbmat, depthmat, depthmatUndistorted, rgbd, rgbd2;
    int histSize = 10;
    float range[] = {0, 1};
    const float* histRange = { range };
    bool uniform = true, accumulate = false;

    Mat g_hist;
    int hist_w = 512, hist_h = 300;
    int bin_w = cvRound( (double) hist_w/histSize );

    //! [loop start]
    while (!protonect_shutdown)
    {
        listener.waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        //! [loop start]

        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);

        // cv::imshow("rgb", rgbmat);
        // cv::imshow("depth", depthmat / 4096.0f);

        //! [registration]
        registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
        //! [registration]

        cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
        cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
        cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);
        cv::Mat depthReal = depthmatUndistorted / 4500.0f;

        cv::imshow("undistorted", depthReal);
        // cv::imshow("registered", rgbd);
        // cv::imshow("depth2RGB", rgbd2 / 4096.0f);
        calcHist(&depthReal, 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate);
        Mat histImage( hist_h, hist_w, CV_32FC3, Scalar( 0, 0, 0 ) );
        normalize(g_hist, g_hist, 10, histImage.rows, NORM_MINMAX, -1, Mat() );
        for( int i = 1; i < histSize; i++ )
        {
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ),
            Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
            Scalar( 0, 0, 255), 2, 8, 0  );
        }
        // std::cout << depthmatUndistorted/4096.0f;
        imshow("calcHist", histImage );

        int key = cv::waitKey(1);
        protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

        //! [loop end]
        listener.release(frames);
    }
    //! [loop end]

    //! [stop]
    dev->stop();
    dev->close();
    //! [stop]

    delete registration;

    std::cout << "Streaming Ends!" << std::endl;
    return 0;
}