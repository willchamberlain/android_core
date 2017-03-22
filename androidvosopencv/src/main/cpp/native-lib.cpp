#include <jni.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

extern "C"
{
void JNICALL Java_william_chamberlain_androidvosopencv_MainActivity_salt(JNIEnv *env, jobject instance,
                                                                           jlong matAddrGray,
                                                                           jint nbrElem) {
    Mat &mGr = *(Mat *) matAddrGray;
    for (int k = 0; k < nbrElem; k++) {
        int i = rand() % mGr.cols;
        int j = rand() % mGr.rows;
        mGr.at<uchar>(j, i) = 255;
    }
}

    void JNICALL Java_william_chamberlain_androidvosopencv_MainActivity_canny(JNIEnv *env, jobject instance,
                                                                       jlong matAddrGray) {
        Mat &mGr = *(Mat *) matAddrGray;
        Canny( mGr, mGr, 50, 150, 3);
}

void rotate_90n(cv::Mat const &src, cv::Mat &dst, int angle)  // see http://stackoverflow.com/questions/16265673/rotate-image-by-90-180-or-270-degrees
{
    CV_Assert(angle % 90 == 0 && angle <= 360 && angle >= -360);
    if(angle == 270 || angle == -90){
        // Rotate clockwise 270 degrees
        cv::transpose(src, dst);
        cv::flip(dst, dst, 0);
    }else if(angle == 180 || angle == -180){
        // Rotate clockwise 180 degrees
        cv::flip(src, dst, -1);
    }else if(angle == 90 || angle == -270){
        // Rotate clockwise 90 degrees
        cv::transpose(src, dst);
        cv::flip(dst, dst, 1);
    }else if(angle == 360 || angle == 0 || angle == -360){
        if(src.data != dst.data){
            src.copyTo(dst);
        }
    }
}


}