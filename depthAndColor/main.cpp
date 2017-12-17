///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2017, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <thread>
#include <limits>
#include <signal.h>

#include <sl/Camera.hpp>

using namespace sl;


// void saveDepthImage() {
// 	float max_value = std::numeric_limits<unsigned short int>::max();
// 	float scale_factor = max_value / zed_
// }
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

int main(int argc, char **argv) {

    // Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_params;
    init_params.depth_mode = DEPTH_MODE_PERFORMANCE; // Use PERFORMANCE depth mode
    init_params.coordinate_units = UNIT_MILLIMETER; // Use millimeter units (for depth measurements)

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS)
        exit(-1);

    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_STANDARD; // Use STANDARD sensing mode

    // Capture 50 images and depth, then stop
    int i = 0;
    sl::Mat image, point_cloud;

    sl::Mat depth(zed.getResolution(), MAT_TYPE_32F_C1);
    sl::Mat depth_disp(zed.getResolution(), MAT_TYPE_8U_C4);

    cv::Mat depth_cv = slMat2cvMat(depth);
    cv::Mat depth_cv_disp = slMat2cvMat(depth_disp);

    cv::namedWindow("depth", 1);
    std::string name = "depth";
    std::string format = ".png";
    while (i < 20) {
        // A new image is available if grab() returns SUCCESS
        if (zed.grab(runtime_parameters) == SUCCESS) {
            // Retrieve left image
            zed.retrieveImage(image, VIEW_LEFT);
            // Retrieve depth map. Depth is aligned on the left image
            zed.retrieveImage(depth_disp, VIEW_DEPTH);

            zed.retrieveMeasure(depth, MEASURE_DEPTH);
            // depth_cv = cv::Mat(depth.getHeight(), depth.getWidth(), CV_8UC4, depth.getPtr<sl::uchar1>(sl::MEM_CPU));
            std::string filename = name + std::to_string(i) + format;

            cv::imwrite(filename, depth_cv);

            cv::imshow("depth", depth_cv_disp);
            char key = cv::waitKey(10);
            // Retrieve colored point cloud. Point cloud is aligned on the left image.
			// zed.retrieveMeasure(point_cloud, MEASURE_XYZRGBA);

            // Get and print distance value in mm at the center of the image
            // We measure the distance camera - object using Euclidean distance
   //          int x = image.getWidth() / 2;
   //          int y = image.getHeight() / 2;
			// sl::float4 point_cloud_value;
			// point_cloud.getValue(x, y, &point_cloud_value);

   //          float distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);
   //          printf("Distance to Camera at (%d, %d): %f mm\n", x, y, distance);

      		// Increment the loop
            i++;
        }
    }
    // Close the camera
    zed.close();
    return 0;
}