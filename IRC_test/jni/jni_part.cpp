
#include <jni.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv/cv.h>
#include <vector>
#include <math.h>

using namespace std;
using namespace cv;

extern "C" {
JNIEXPORT void JNICALL Java_org_opencv_samples_tutorial2_Tutorial2Activity_FindFeatures(JNIEnv*, jobject, jlong addrGray, jlong addrRgba);

JNIEXPORT void JNICALL Java_org_opencv_samples_tutorial2_Tutorial2Activity_FindFeatures(JNIEnv*, jobject, jlong addrGray, jlong addrRgba)
{
    Mat& mGr  = *(Mat*)addrGray;
    Mat& mRgb = *(Mat*)addrRgba;

    vector<Vec3f> circles;
    GaussianBlur( mGr, mGr, Size(9, 9), 2, 2 );
    HoughCircles(mGr, circles, CV_HOUGH_GRADIENT,1, 20, 200, 100 );
    for( int i = 0; i < circles.size(); i++ )
    {
         Point center(static_cast<int>(circles[i][0]), static_cast<int>(circles[i][1]));
         int radius = static_cast<int>(circles[i][2]);
         // draw the circle center
         circle( mRgb, center, 3, Scalar(0,255,0), -1, 8, 0 );
         // draw the circle outline
         circle( mRgb, center, radius, Scalar(0,0,255), 3, 8, 0 );
    }
}
}

