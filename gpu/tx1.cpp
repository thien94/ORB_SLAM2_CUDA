#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <System.h>

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
    if(argc != 3) {
        cerr << endl << "Usage: " << argv[0] << " [path to vocabulary] [path to settings]" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    const char * gst = "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)960, height=(int)540, format=(string)I420, framerate=(fraction)10/1 ! nvvidconv flip-method=2 ! videoconvert ! appsink";
    cv::VideoCapture cap(gst);
    if (!cap.isOpened()) {
      printf("can not open camera or video file\n%s", gst);
      return -1;
    }

    // Main loop
    SET_CLOCK(t0);
    cv::Mat im;
    int frameNumber = 0;
    while (true) {
      ++frameNumber;
      cap >> im;
      SET_CLOCK(t1);
      double tframe = TIME_DIFF(t1, t0);
      //if (tframe > 60) break;
      // Pass the image to the SLAM system
      SLAM.TrackMonocular(im,tframe);
      SET_CLOCK(t2);
      cerr << frameNumber << ": " << tframe << " " << TIME_DIFF(t2, t1) << "\n";
    }

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}
