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


void translationRotationWithoutAxisChange(double m_tagSize, double m_fx, double m_fy, double m_px,
                                          double m_py,
                                          const vector<AprilTags::TagDetection> &detections, int i,
                                          Eigen::Vector3d &translation, Eigen::Matrix3d &rotation);

using namespace std;

void translationRotationWithoutAxisChange(double m_tagSize, double m_fx, double m_fy, double m_px,
                                          double m_py,
                                          AprilTags::TagDetection &tagDetection,
                                          Eigen::Vector3d &translation, Eigen::Matrix3d &rotation) {
    Eigen::Matrix4d transform = tagDetection.getRelativeTransform(m_tagSize, m_fx, m_fy, m_px, m_py);  // see /mnt/nixbig/build_workspaces/apriltags_ros/src/apriltags_ros/apriltags_ros/src/apriltag_detector.cpp
    translation[0] = transform(0, 3);
    translation[1] = transform(1, 3);
    translation[2] = transform(2, 3);
    rotation = transform.block(0, 0, 3, 3);
}


void translationRotationWithAxisChange(double m_tagSize, double m_fx, double m_fy, double m_px,
                                          double m_py,
                                          AprilTags::TagDetection &tagDetection,
                                          Eigen::Vector3d &translation, Eigen::Matrix3d &rotation) {
    tagDetection.getRelativeTranslationRotation(m_tagSize,m_fx,m_fy,m_px,m_py,translation,rotation);
}

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
                                                                            jlong tagDetectorPointer,    // pointer to an AprilTags::TagDetector instance
                                                                            jdouble tagSize_metres,
                                                                            jdouble fx_pixels,
                                                                            jdouble fy_pixels
    ) {
        AprilTags::TagDetector *m_tagDetector = (AprilTags::TagDetector *)tagDetectorPointer;
//        AprilTags::TagCodes m_tagCodes;
//        m_tagCodes = AprilTags::tagCodes36h11;
//        m_tagDetector = new AprilTags::TagDetector(m_tagCodes);
//        m_tagDetector = new AprilTags::TagDetector(AprilTags::tagCodes36h11);

        Mat &mGr = *(Mat *) matAddrGray;
        Mat &mRgb = *(Mat *) matAddrRgb;
        int m_width = mGr.cols;    // image size in pixels
        int m_height = mGr.rows;
        double m_tagSize = tagSize_metres;  // April tag side length in meters of square black frame
        // linear camera model
        double m_fx = fx_pixels; // TODO - check calibration - camera focal length in pixels - http://ksimek.github.io/2013/08/13/intrinsic/ - https://en.wikipedia.org/wiki/Camera_matrix
        double m_fy = fy_pixels; // TODO - check calibration - see https://cloudstor.aarnet.edu.au/plus/index.php/apps/files/?dir=%2Fproject_AA1__1_1%2Fresults%2F2016_12_04_callibrate_in_ROS%2Fcalibrationdata_131#editor
        // m_fx = 519.902859;       // TODO - for Samsung Galaxy S3s from /mnt/nixbig/ownCloud/project_AA1__1_1/results/2016_12_04_callibrate_in_ROS/calibrationdata_grey/ost.txt
        //m_fy = 518.952669;
        double m_px = m_width/2;   // TODO - use proper camera principal point
        double m_py = m_height/2;
        Eigen::Vector3d translation;    // translation vector from camera to the April tag - see TagDetection.cc
        Eigen::Matrix3d rotation;       // orientation of April tag with respect to camera - see TagDetection.cc
        Eigen::Vector3d rollPitchYaw;
        Eigen::Quaterniond quaternion;
        const std::string quaternion_format_as_string = std::string("tag %d at x=%.4f y=%.4f z=%.4f roll=%.4f pitch=%.4f yaw=%.4f qx=%.4f qy=%.4f qz=%.4f qw=%.4f");
        const char* quaternion_format_as_string_c_str = quaternion_format_as_string.c_str();

        vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(mGr);

        jstring         str;
        jobjectArray    tags_as_strings = 0;
        jsize           detections_size = detections.size();
        int             detections_size_int = detections.size();
        tags_as_strings = (jobjectArray)env->NewObjectArray(detections_size,(env)->FindClass("java/lang/String"),env->NewStringUTF(""));

//        Eigen::Matrix3d rotationReflectZ;
//        rotationReflectZ << 1.0, 0.0, 0.0,
//                0.0, 1.0, 0.0,
//                0.0, 0.0, -1.0;

        for (int i=0; i<detections.size(); i++) {
            // NOTE: inverse - camera pose relative to tag - would be the inverse rotation then the inverse translation, I think
            // robotic to opencv camera
            // -90 Z then -90 X
            // or 90 Y then -90 Z

            // from getRelativeTransform tag to robot, with x toward robot, and Z upward, and Y toward robot right
            //

            // for _relative orientation of tag
// 2017_03_03           detections[i].getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py, translation, rotation);
//            Eigen::Matrix3d m;
//            m <<  1, 0, 0,
//                  0,-1, 0,
//                  0, 0, 1;
//            Eigen::Matrix3d rotationRotated;
//            rotationRotated = m*rotation;
//            rotation = rotationRotated;

            // for _non-relative orientation of tag
//            Eigen::Vector3d unchangedTranslation;
//            Eigen::Matrix3d unchangedRotation;

//            translationRotationWithoutAxisChange(m_tagSize, m_fx, m_fy, m_px, m_py, detections[i], translation, rotation);
            translationRotationWithAxisChange(m_tagSize, m_fx, m_fy, m_px, m_py, detections[i], translation, rotation);

//            Eigen::Matrix3d m;
//            m = Eigen::AngleAxisd(3.142, Eigen::Vector3d::UnitZ());
//            Eigen::Matrix3d rotationRotated = m*unchangedRotation;
//            rotation = rotationRotated;

            // for _relative orientation of tag
//            Eigen::Matrix4d transform = detections[i].getRelativeTransform(m_tagSize, m_fx, m_fy, m_px, m_py);  // see /mnt/nixbig/build_workspaces/apriltags_ros/src/apriltags_ros/apriltags_ros/src/apriltag_detector.cpp
//            rotation = transform.block(0, 0, 3, 3);

//            rotation = rotationReflectZ*rotation;

            quaternion = Eigen::Quaternion<double>(rotation);

//            detections[i].getRelativeTranslationRotationQuaternion(
//                    m_tagSize,
//                    m_fx, m_fy,
//                    m_px, m_py,
//                    translation, rotation, quaternion);
            // also highlight in the image
            //detections[i].draw(mRgb, translation, rotation); // my code - label the tag with the x,y,z translation from the camera - clutters the image
            detections[i].draw(mRgb);
            rollPitchYaw = rotation.eulerAngles(0, 2, 1);
            Eigen::Quaterniond& q = quaternion;  // q is alias for quaternion

            // Individual angles, rather than Euler angles, see http://stackoverflow.com/a/37560411/1200764
            double bank     = atan2(2.0 * (q.x() * q.y() + q.w() * q.x()) , 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));        // isolated roll
            double attitude = asin(2.0 * (q.y() * q.w() - q.x() * q.x()));    // isolated pitch
            double heading  = atan2(2.0 * (q.x() * q.w() + q.x() * q.y()) , - 1.0 + 2.0 * (q.w() * q.w() + q.x() * q.x()));        // isolated yaw
            LOGI("  native: --  bank=%.4f, attitude=%.4f, heading=%.4f",180*bank/3.14159265,180*attitude/3.14159265,180*heading/3.14159265);
            // see http://stackoverflow.com/a/18115837/1200764
            bank = atan2(2.0*(q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
            heading = atan2(2.0*(q.y()*q.z() + q.w()*q.x()), q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z());
            attitude = asin(-2.0*(q.x()*q.z() - q.w()*q.y()));
            LOGI("  native: --  bank=%.4f, attitude=%.4f, heading=%.4f",180*bank/3.14159265,180*attitude/3.14159265,180*heading/3.14159265);

            LOGI( quaternion_format_as_string_c_str,
                  detections[i].id, translation(0), translation(1), translation(2),
                  rollPitchYaw(0), rollPitchYaw(1), rollPitchYaw(2),
                    quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w() );
            LOGI("  native: --  rpy degrees:  roll=%.4f pitch=%.4f yaw=%.4f", rollPitchYaw(0)*180.0/3.14159265, rollPitchYaw(1)*180.0/3.14159265, rollPitchYaw(2)*180.0/3.14159265 );
            std::cout << "tag " << detections[i].id << " at x=" << translation(0) << " y=" << translation(1) << " z=" << translation(2) << std::endl;
            std::cout.flush();
//            detections[i].draw(mRgb);
            char tag_and_pose_data[200];
            sprintf(tag_and_pose_data, quaternion_format_as_string_c_str,
                    detections[i].id, translation(0), translation(1), translation(2),
                    rollPitchYaw(0), rollPitchYaw(1), rollPitchYaw(2),
                    quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w() );

            str = (env)->NewStringUTF(tag_and_pose_data);
            (env)->SetObjectArrayElement(tags_as_strings, i, str);
        }
//        rotate_90n(mRgb,mRgb,270);
        std::ostringstream strStx;
        strStx << "0,0";
        cv::putText(mRgb, strStx.str(),
                    cv::Point2f(0, 0),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(220,220,255));
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