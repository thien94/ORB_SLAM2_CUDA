/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include"System.h"

using namespace std;

#ifdef COMPILEDWITHC11
#define SET_CLOCK(t0) \
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
#else
#define SET_CLOCK(t0) \
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

#define TIME_DIFF(t1, t0) \
        std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count()

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: " << argv[0] << " [path to vocabulary] [path to settings]" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    //const char * gst = "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)I420, framerate=(fraction)120/1 ! nvvidconv flip-method=2 ! videoconvert ! appsink";
    const char * gst = "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)960, height=(int)540, format=(string)I420, framerate=(fraction)10/1 ! nvvidconv flip-method=2 ! videoconvert ! appsink";
    //const char * gst = "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)I420, framerate=(fraction)24/1 ! nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    //const char * gst = "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)I420, framerate=(fraction)30/1 ! nvtee ! nvvidconv flip-method=2 ! video/x-raw(memory:NVMM), format=(string)I420 ! appsink";
    cv::VideoCapture capture(gst);
    if (!capture.isOpened()) {
      printf("can not open camera or video file\n%s", gst);
      return -1;
    }

    SET_CLOCK(t0);
    // Main loop
    cv::Mat im;

    /*
    cv::namedWindow("im", cv::WINDOW_AUTOSIZE);
    while (true) {
      capture >> im;
      if (!im.empty()) {
        cv::imshow("im", im);
      }
      if (cv::waitKey(25) != -1) {
        break;
      }
    }
    */

    while (true) {
        capture >> im;
        SET_CLOCK(t1);
        double tframe = TIME_DIFF(t1, t0);
        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);
    }

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}
