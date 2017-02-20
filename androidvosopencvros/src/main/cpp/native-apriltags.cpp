#include <jni.h>
#include <string>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <AprilTags/TagDetection.h>
#include <AprilTags/TagDetector.h>
#include "AprilTags/Tag36h11.h"

#include <android/log.h>

#define  LOG_TAG    "native-apriltags"

#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#define  LOGW(...)  __android_log_print(ANDROID_LOG_WARN,LOG_TAG,__VA_ARGS__)
#define  LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG,LOG_TAG,__VA_ARGS__)
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)


using namespace std;
using namespace cv;



extern "C"
{
    void rotate_90n(cv::Mat const &src, cv::Mat &dst, int angle);

    jlong JNICALL Java_william_chamberlain_androidvosopencvros_MainActivity_newTagDetector(JNIEnv *, jobject) {
        return (long)(new AprilTags::TagDetector(AprilTags::tagCodes36h11));
    }

    void JNICALL Java_william_chamberlain_androidvosopencvros_MainActivity_deleteTagDetector(JNIEnv *, jobject, jlong tagDetectorPointer) {
        delete (AprilTags::TagDetector *)tagDetectorPointer;
    }

    jobjectArray JNIEXPORT Java_william_chamberlain_androidvosopencvros_MainActivity_aprilTags(JNIEnv *env, jobject instance,
                                                                            jlong matAddrGray,          // pointer to grayscale image matrix - input to Apriltags detection
                                                                            jlong matAddrRgb,           // pointer to RGB image matrix - input and output
                                                                            jlong tagDetectorPointer    // pointer to an AprilTags::TagDetector instance
        ) {
        AprilTags::TagDetector *m_tagDetector = (AprilTags::TagDetector *)tagDetectorPointer;
//        AprilTags::TagCodes m_tagCodes;
//        m_tagCodes = AprilTags::tagCodes36h11;
//        m_tagDetector = new AprilTags::TagDetector(m_tagCodes);
//        m_tagDetector = new AprilTags::TagDetector(AprilTags::tagCodes36h11);

        Mat &mGr = *(Mat *) matAddrGray;
        Mat &mRgb = *(Mat *) matAddrRgb;
        int m_width = mGr.cols; // image size in pixels
        int m_height = mGr.rows;
        double m_tagSize = 0.168; // April tag side length in meters of square black frame
        // linear camera model
        double m_fx = 1098.002914; // camera focal length in pixels - http://ksimek.github.io/2013/08/13/intrinsic/ - https://en.wikipedia.org/wiki/Camera_matrix
        double m_fy = 1096.498477; // see https://cloudstor.aarnet.edu.au/plus/index.php/apps/files/?dir=%2Fproject_AA1__1_1%2Fresults%2F2016_12_04_callibrate_in_ROS%2Fcalibrationdata_131#editor
        double m_px = m_width/2; // camera principal point
        double m_py = m_height/2;
        Eigen::Vector3d translation;    // translation vector from camera to the April tag - see TagDetection.cc
        Eigen::Matrix3d rotation;       // orientation of April tag with respect to camera - see TagDetection.cc
        Eigen::Vector3d rollPitchYaw;
        Eigen::Quaterniond quaternion;
        const std::string quaternion_format_as_string = std::string("tag %d at x=%.4f y=%.4f z=%.4f roll=%.4f pitch=%.4f yaw=%.4f qx=%.4f qy=%.4f qz=%.4f qw=%.4f");
        const char* quaternion_format_as_string_c_str = quaternion_format_as_string.c_str();

        vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(mGr);
        for (int i=0; i<detections.size(); i++) {
            // NOTE: inverse - camera pose relative to tag - would be the inverse rotation then the inverse translation, I think
            detections[i].getRelativeTranslationRotationQuaternion(
                    m_tagSize,
                    m_fx, m_fy,
                    m_px, m_py,
                    translation, rotation, quaternion);
            // also highlight in the image
            //detections[i].draw(mRgb, translation, rotation); // my code - label the tag with the x,y,z translation from the camera - clutters the image
            detections[i].draw(mRgb);
            rollPitchYaw = rotation.eulerAngles(0, 2, 1);
            LOGI( quaternion_format_as_string_c_str,
                  detections[i].id, translation(0), translation(1), translation(2),
                  rollPitchYaw(0), rollPitchYaw(1), rollPitchYaw(2),
                    quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w() );
            std::cout << "tag " << detections[i].id << " at x=" << translation(0) << " y=" << translation(1) << " z=" << translation(2) << std::endl;
//            detections[i].draw(mRgb);
        }
//        rotate_90n(mRgb,mRgb,270);
        std::ostringstream strStx;
        strStx << "0,0";
        cv::putText(mRgb, strStx.str(),
                    cv::Point2f(0, 0),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(220,220,255));

        jstring         str;
        jobjectArray    tags_as_strings = 0;
        jsize           detections_size = detections.size();
        int             detections_size_int = detections.size();
        tags_as_strings = (jobjectArray)env->NewObjectArray(detections_size,(env)->FindClass("java/lang/String"),env->NewStringUTF(""));
        for(int i=0; i<detections_size_int; i++) {
            // NOTE: inverse - camera pose relative to tag - would be the inverse rotation then the inverse translation, I think
            detections[i].getRelativeTranslationRotationQuaternion(
                    m_tagSize,
                    m_fx, m_fy,
                    m_px, m_py,
                    translation, rotation, quaternion);
            char tag_and_pose_data[200];
            sprintf(tag_and_pose_data, quaternion_format_as_string_c_str,
                    detections[i].id, translation(0), translation(1), translation(2),
                    rollPitchYaw(0), rollPitchYaw(1), rollPitchYaw(2));

            str = (env)->NewStringUTF(tag_and_pose_data);
            (env)->SetObjectArrayElement(tags_as_strings, i, str);
        }
        return tags_as_strings;
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