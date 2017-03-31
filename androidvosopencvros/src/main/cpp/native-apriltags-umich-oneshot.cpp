#include <jni.h>
#include <string>
#include <sstream>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/opencv.hpp"

#include <android/log.h>
#include <common/image_u8.h>
#include <apriltag.h>

#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "tag16h5.h"

#include "common/getopt.h"

#define  LOG_TAG    "native-apriltags-umich-oneshot"

#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#define  LOGW(...)  __android_log_print(ANDROID_LOG_WARN,LOG_TAG,__VA_ARGS__)
#define  LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG,LOG_TAG,__VA_ARGS__)
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)


using namespace cv;

// from android samples
/* return current time in milliseconds */
static double now_ms(void) {
    struct timespec res;
    clock_gettime(CLOCK_REALTIME, &res);
    return 1000.0 * res.tv_sec + (double) res.tv_nsec / 1e6;

}
static double now_us(void) {
    struct timespec res;
    clock_gettime(CLOCK_REALTIME, &res);
    return 1000000.0 * res.tv_sec + (double) res.tv_nsec / 1e3;

}


extern "C"
{

// draw one April tag detection on actual image
void draw(cv::Mat& image, const int id, const double xc, const double yc, const double x1, const double y1, const double x2, const double y2, const double x3, const double y3, const double x4, const double y4 )
     {
    std::pair<float, float> cxy = std::pair<float, float>(xc,yc);

    // use corner points detected by line intersection
    std::pair<float, float> p1 = std::pair<float, float>(x1,y1);
    std::pair<float, float> p2 = std::pair<float, float>(x2,y2);
    std::pair<float, float> p3 = std::pair<float, float>(x3,y3);
    std::pair<float, float> p4 = std::pair<float, float>(x4,y4);

    // plot outline
    cv::line(image, cv::Point2f(p1.first, p1.second), cv::Point2f(p2.first, p2.second), cv::Scalar(255,0,0,0) );
    cv::line(image, cv::Point2f(p2.first, p2.second), cv::Point2f(p3.first, p3.second), cv::Scalar(0,255,0,0) );
    cv::line(image, cv::Point2f(p3.first, p3.second), cv::Point2f(p4.first, p4.second), cv::Scalar(0,0,255,0) );
    cv::line(image, cv::Point2f(p4.first, p4.second), cv::Point2f(p1.first, p1.second), cv::Scalar(255,0,255,0) );

    // mark center
    cv::circle(image, cv::Point2f(cxy.first, cxy.second), 8, cv::Scalar(0,0,255,0), 2);

    // print ID
    std::ostringstream strSt;
    strSt << "#" << id;
    cv::putText(image, strSt.str(),
                cv::Point2f(cxy.first + 10, cxy.second + 10),
                cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));
}


//    #include    "jpeglib.h"
    // First, just call the function to get the timing
    // Second, call it and bring back the tag values
    // Third, get the tag pose and get the timing

    // see apriltag_v4l_integration_demo.c for example of invocation
    jlong JNICALL Java_william_chamberlain_androidvosopencvros_MainActivity_aprilTagsUmichOneShot(JNIEnv *env, jobject instance,
                                            jlong matAddrGray,              // pointer to grayscale image matrix - input to Apriltags detection
                                            jlong matAddrRgb,               // pointer to RGB image matrix - input and output
                                            jlong tagDetectorPointer,       // pointer to an AprilTags::TagDetector instance
                                            jdouble tagSize_metres,
                                            jdouble fx_pixels,
                                            jdouble fy_pixels               ){
    double start_ms = now_ms(); // start time
    double time_then = start_ms;
    double time_now = time_then;
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmichOneShot: START %f ms.", (time_now-start_ms));

        //// start from main()

        getopt_t *getopt = getopt_create();
        time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmichOneShot: getopt_create(): %f ms.", (time_now-time_then));

        getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
        getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
        getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
        getopt_add_string(getopt, 'f', "family", "tag16h5", "Tag family to use");
        getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
        getopt_add_int(getopt, 't', "threads", "8", "Use this many CPU threads");
        getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
        getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input; negative sharpens");
        getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
        getopt_add_bool(getopt, '1', "refine-decode", 0, "Spend more time trying to decode tags");
        getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");

        time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmichOneShot: getopt_add_bool, etc: %f ms.", (time_now-time_then));

        const zarray_t *inputs = getopt_get_extra_args(getopt);

        apriltag_family_t *tagFamily = NULL;
        const char *famname = "tag16h5";
        if (!strcmp(famname, "tag36h11"))
            tagFamily = tag36h11_create();
        else if (!strcmp(famname, "tag36h10"))
            tagFamily = tag36h10_create();
        else if (!strcmp(famname, "tag36artoolkit"))
            tagFamily = tag36artoolkit_create();
        else if (!strcmp(famname, "tag25h9"))
            tagFamily = tag25h9_create();
        else if (!strcmp(famname, "tag25h7"))
            tagFamily = tag25h7_create();
        else if (!strcmp(famname, "tag16h5"))
            tagFamily = tag16h5_create();
        else {
            printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
            LOGI("MainActivity_aprilTagsUmichOneShot: Unrecognized tag family name: %s .",famname);
            exit(-1);
        }
        time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmichOneShot: setting up tag family: %f ms.", (time_now-time_then));

        tagFamily->black_border = getopt_get_int(getopt, "border");
        LOGI("MainActivity_aprilTagsUmichOneShot: tagFamily='%s' black_border=%d .", tagFamily->name, tagFamily->black_border);

        apriltag_detector_t *tagDetector = apriltag_detector_create();              //// DOING TODO - move outside to re-use; same as for Troy's tracker
        time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmichOneShot: created apriltag_detector_t: %f ms.", (time_now-time_then));
        apriltag_detector_add_family(tagDetector, tagFamily);
        time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmichOneShot: added tag family: %f ms.", (time_now-time_then));
        time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmichOneShot: tagDetector: %f ms.", (time_now-time_then));
        tagDetector->quad_decimate = getopt_get_double(getopt, "decimate");
        tagDetector->quad_sigma = getopt_get_double(getopt, "blur");
        tagDetector->nthreads = getopt_get_int(getopt, "threads");
        tagDetector->debug = getopt_get_bool(getopt, "debug");
        tagDetector->refine_edges = getopt_get_bool(getopt, "refine-edges");
        tagDetector->refine_decode = getopt_get_bool(getopt, "refine-decode");
        tagDetector->refine_pose = getopt_get_bool(getopt, "refine-pose");

        time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmichOneShot: tagDetector set attributes: %f ms.", (time_now-time_then));

    LOGI("MainActivity_aprilTagsUmichOneShot: tagDetector has %d tag families loaded", tagDetector->tag_families->size );


    Mat &mGr = *(Mat *) matAddrGray;
    int width = mGr.cols;       // image size in pixels
    int height = mGr.rows;      // image size in pixels
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmichOneShot: Mat &mGr = *(Mat *) matAddrGray: %f ms.", (time_now-time_then));

//    int quiet = getopt_get_bool(getopt, "quiet");
//
//    int maxiters = getopt_get_int(getopt, "iters");

    const int hamm_hist_max = 10;
    int hamm_hist[hamm_hist_max];
    memset(hamm_hist, 0, sizeof(hamm_hist));
    int total_quads = 0;
    int total_hamm_hist[hamm_hist_max];
    memset(total_hamm_hist, 0, sizeof(total_hamm_hist));
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmichOneShot: create hamm and hamm_hist_max: %f ms.", (time_now-time_then));

    image_u8_t *im = NULL;
    im = image_u8_create(width, height);
    for (int row_ = 0; row_ < height; row_++) {
        for (int col_ = 0; col_ < width; col_++) {
            im->buf[row_*im->stride + col_ ] = mGr.at<uchar>(row_,col_);  // yth row (from 0), xth pixel (from 0), greyscale so only one channel
        }
    }
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmichOneShot: image bytes to image_u8_t: %f ms.", (time_now-time_then));

    zarray_t *detections = apriltag_detector_detect(tagDetector, im);  ////   the meaty bit - run tag detection on a grayscale image given
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmichOneShot: do the detections: %f ms.", (time_now-time_then));

    int num_tags_detected = zarray_size(detections);
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        LOGI("detection %3d: id (%2dx%2d)-%-4d, hamming %d, goodness %8.3f, margin %8.3f, centre pixel (%8.3f,%8.3f)\n",
                   i, det->family->d*det->family->d, det->family->h, det->id, det->hamming, det->goodness, det->decision_margin
                    , det->c[0],det->c[1]);
        LOGI( "\t\tcorners X = [%5.0f, %5.0f, %5.0f, %5.0f]\n" , det->p[0][0], det->p[1][0], det->p[2][0], det->p[3][0] );
        LOGI( "\t\tcorners y = [%5.0f, %5.0f, %5.0f, %5.0f]\n" , det->p[0][1], det->p[1][1], det->p[2][1], det->p[3][1] );
        LOGI( " hold on; plot([%5.0f], [%5.0f],'+','LineWidth',2);\n" , det->c[0],det->c[1] );
        LOGI( " hold on; plot([%5.0f, %5.0f, %5.0f, %5.0f, %5.0f]" , det->p[0][0], det->p[1][0], det->p[2][0], det->p[3][0], det->p[0][0] );
        LOGI( ", [%5.0f, %5.0f, %5.0f, %5.0f, %5.0f],'--+','LineWidth',2);\n\n" , det->p[0][1], det->p[1][1], det->p[2][1], det->p[3][1], det->p[0][1] );
        hamm_hist[det->hamming]++;
        total_hamm_hist[det->hamming]++;
        time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmichOneShot: list tag detection attributes: %f ms  for tag detection %d.", (time_now-time_then), i);

        Mat &matRgb = *(Mat *) matAddrRgb;
        draw(matRgb, det->id, det->c[0],det->c[1], det->p[0][0], det->p[0][1], det->p[1][0], det->p[1][1], det->p[2][0], det->p[2][1], det->p[3][0], det->p[3][1]);
        time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmichOneShot: draw(matRgb, ...): %f ms.", (time_now-time_then));
    }

    apriltag_detections_destroy(detections);
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmichOneShot: apriltag_detections_destroy: %f ms.", (time_now-time_then));

    total_quads += tagDetector->nquads;

    image_u8_destroy(im);
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmichOneShot: image_u8_destroy(im): %f ms.", (time_now-time_then));

    apriltag_detector_destroy(tagDetector);
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmichOneShot: apriltag_detector_destroy(tagDetector): %f ms.", (time_now-time_then));

    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmichOneShot: tag36h11_destroy(tagFamily): %f ms.", (time_now-time_then));

    LOGI("MainActivity_aprilTagsUmichOneShot: returning %d as number of detections",num_tags_detected);
    return num_tags_detected;
    }


}



