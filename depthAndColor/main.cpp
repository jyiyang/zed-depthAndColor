#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <thread>
#include <limits>
#include <signal.h>
#include <getopt.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctime>

#include <sl/Camera.hpp>

#include "Logger.h"

using namespace sl;

cv::Mat slMat2cvMat(Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}

void printHelp() {
	fprintf(stderr, "./depthAndColor -d [recording duration in seconds] -f [output filename end with .klg]\n");
}

int main(int argc, char *argv[]) {

	int option = 0;
	int duration = 0;
	char *filename;

	while ((option = getopt(argc, argv, "hf:d:")) != -1) {
		switch (option)
		{
			case 'f':
				filename = optarg;
				break;
			case 'd':
				duration = atoi(optarg);
				break;
			case 'h':
				printHelp();
				exit(1);
			case '?':
				if (optopt == 'f') {
					fprintf(stderr, "Option -%c requires an argument in string.\n", optopt);
					printHelp();
				}
				else if (optopt == 'd') {
					fprintf(stderr, "Option -%c requires an argument in integer.\n", optopt);
					printHelp();
				}
				else if (isprint(optopt)) {
					fprintf(stderr, "Unknown option -%c.\n", optopt);
					printHelp();
				}
				else {
					fprintf(stderr, "Unknown option character \\x%x.\n", optopt);
					printHelp();
				}
				exit(1);
			default:
				printHelp();
				exit(1);
		}
	}

	printf("Duration: %d, Filename: %s\n", duration, filename);
    // Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_params;
    init_params.depth_mode = DEPTH_MODE_QUALITY; // Use QUALITY depth mode
    init_params.coordinate_units = UNIT_MILLIMETER; // Use millimeter units (for depth measurements)

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS)
        exit(-1);

    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_STANDARD; // Use STANDARD sensing mode

    // Initialize klg logger
    Logger logger(filename, true);

    int numFrames = 0;
    sl::Mat leftImage(zed.getResolution(), MAT_TYPE_8U_C4);

    sl::Mat depth(zed.getResolution(), MAT_TYPE_32F_C1);
    sl::Mat depth_disp(zed.getResolution(), MAT_TYPE_8U_C4);

    cv::Mat depth_cv = slMat2cvMat(depth);
    cv::Mat depth_cv_disp = slMat2cvMat(depth_disp);
    cv::Mat depth_cv_ushort;
    cv::Mat color_cv = slMat2cvMat(leftImage);
    cv::Mat color_cv_3c;

    // Initialize timer
    bool endFlag = false;
    clock_t startTime = clock();

    while (!endFlag) {
        // A new image is available if grab() returns SUCCESS
        if (zed.grab(runtime_parameters) == SUCCESS) {
            // Retrieve left image
            zed.retrieveImage(leftImage, VIEW_LEFT);
            zed.retrieveMeasure(depth, MEASURE_DEPTH);

            depth_cv.convertTo(depth_cv_ushort, CV_16UC1);
            color_cv.convertTo(color_cv_3c, CV_8UC3);

            // Write it to the logger
            logger.writeFrame(depth_cv_ushort, color_cv_3c, numFrames+1);

      		// Increment the loop
      		++numFrames;
            printf("Currently in frame %d\n", numFrames);
            

            clock_t timePassed = clock();
            if ( (int)((float)timePassed/CLOCKS_PER_SEC) > duration) {
            	printf("Cleaning up with a total of %d frames...\n", numFrames);
            	endFlag = true;
            }
        }
    }

    // Close the camera
    zed.close();
    // Close the logger
    logger.close();
    return 0;
}
