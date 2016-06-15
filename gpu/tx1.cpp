#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <System.h>
#include <Utils.hpp>

using namespace std;

#ifdef COMPILEDWITHC11
#define SET_CLOCK(t0) \
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
#else
#define SET_CLOCK(t0) \
        std::chrono::monotonic_clock::time_point t0 = std::chrono::monotonic_clock::now();
#endif

#define TIME_DIFF(t1, t0) \
        (std::chrono::duration_cast<std::chrono::duration<double>>((t1) - (t0)).count())

int main(int argc, char **argv)
{
    if(argc != 4) {
        cerr << endl << "Usage: " << argv[0] << " [path to vocabulary] [path to settings] [seconds to run]" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // const char * gst = "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)I420, framerate=(fraction)24/1 ! nvvidconv flip-method=2 ! videoconvert ! appsink";
    const char * gst = "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)960, height=(int)540, format=(string)I420, framerate=(fraction)30/1 ! nvvidconv flip-method=2 ! videoconvert ! appsink";
    cv::VideoCapture cap(gst);
    if (!cap.isOpened()) {
      printf("can not open camera or video file\n%s", gst);
      return -1;
    }

    double ttl = atof(argv[3]);

    double tsum = 0;
    double tbuf[10] = {0.0};
    int tpos = 0;
    double trackTimeSum = 0.0;
    // Main loop
    cv::Mat im;
    SET_CLOCK(t0);
    int frameNumber = 0;
    while (true) {
      cap >> im;
      if (im.empty()) continue;
      SET_CLOCK(t1);
      double tframe = TIME_DIFF(t1, t0);
      if (tframe > ttl) {
        break;
      }

      PUSH_RANGE("Track image", 4);
      // Pass the image to the SLAM system
      SLAM.TrackMonocular(im,tframe);
      POP_RANGE;
      SET_CLOCK(t2);

      double trackTime = TIME_DIFF(t2, t1);
      trackTimeSum += trackTime;
      tsum = tframe - tbuf[tpos];
      tbuf[tpos] = tframe;
      tpos = (tpos + 1) % 10;
      cerr << "Frame " << frameNumber << " : " << tframe << " " << trackTime << " " << 10 / tsum << "\n";
      ++frameNumber;
    }
    // Stop all threads
    SLAM.Shutdown();

    cerr << "Mean track time: " << trackTimeSum / frameNumber << " , mean fps: " << frameNumber / ttl << "\n";

    return 0;
}
