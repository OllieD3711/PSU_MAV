#ifndef IMAGE_H
#define IMAGE_H

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <stdio.h>
#include <iostream>
    
#include <vector> 
#include <numeric>

#include <raspicam/raspicam_cv.h>

#include <stdint.h>

#include <sys/time.h>

#include <math.h>
#include <thread>

#include <unistd.h>

#include "struct.h"

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "apriltag_pose.h"
#include "common/getopt.h"
}

#define APRILSIZE 149.5 // april tag side length (mm)
#define FOCALX 100      // horizontal camera focal length (pixels)
#define FOCALY 100      // vertical camera focal length (pixels)
#define PRINCX 0        // Principle point in x (in pixels)
#define PRINCY 0        // Principle point in y (in pixels)

using namespace cv;
using namespace std;

extern Buffers buffers;

//------------------------------------------------------------------------------
 

class Image{
	
	raspicam::RaspiCam_Cv cap;
	
	float theta;        // Camera half-angle in x-direction (640 pixels)
	float phi ;          // Camera half-angle in y-direction (480 pixels)
	int width;      // Maximum x-axis grid location
	int height;      // Maximum y-axis grid location
	
	ImgData imgData;

	std::thread capThread,boundThread,mainThread;
	bool capThreadActive,boundThreadActive,mainThreadActive;
	
	bool getBoundary,getHomeBase,getTarget,getPickup;

        apriltag_family_t *tf = NULL;
        apriltag_detector_t *td = NULL;

	uint64_t getTimeUsec(){
		struct timespec t;
		clock_gettime(CLOCK_REALTIME,&t);
		return (t.tv_sec)*1e6+(t.tv_nsec)/1e3;
	}

        typedef struct targetType{
            bool isHelipad;
            bool isFerry;
            bool isBullseye;
            bool isPickUp;
        }targetType;

        typedef struct tagPoseInfo{
            apriltag_detection_t Info;
            apriltag_pose_t Pose;
        }tagPoseInfo;

public:

	Image(int width, int height);

	void startProcessing();
	void stopProcessing();
	void startCapture();
	void stopCapture();
	void capture();
	void process();
	cv::Mat detectBoundary(cv::Mat, float, int &, float &);
        cv::Mat Image::detectApril(cv::Mat img, apriltag_pose_t *pose);
        void calibrate(Scalar &u, Scalar &l);
        void targetType(apriltag_pose_t *pose);
        void detTargetType(apriltag_detection_t *det, targetType *tt);
        cv::Mat detectHomeBase(cv::Mat imgOriginal, float altitude, int &direction, float &x, float &y);
	cv::Mat detectTarget(cv::Mat imgOriginal, float altitude, int &direction, float &x, float &y);
	void setType(bool a, bool b, bool c, bool d);

	ImgData getImgData(){
		return imgData;
	}

	~Image(){
		stopProcessing();
		stopCapture();
	}
};

#endif
