#include <jni.h>
#include <string>
#include <sstream>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <android/log.h>
#include <common/image_u8.h>
#include <apriltag.h>

#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"

#include "common/getopt.h"

#define  LOG_TAG    "native-apriltags-umich"

#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#define  LOGW(...)  __android_log_print(ANDROID_LOG_WARN,LOG_TAG,__VA_ARGS__)
#define  LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG,LOG_TAG,__VA_ARGS__)
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)

apriltag_detector_t *createDetector(const apriltag_family_t *tagFamily);

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
//    #include    "jpeglib.h"
    // First, just call the function to get the timing
    // Second, call it and bring back the tag values
    // Third, get the tag pose and get the timing

    // see apriltag_v4l_integration_demo.c for example of invocation
    jlong JNICALL Java_william_chamberlain_androidvosopencvros_MainActivity_aprilTagsUmich(JNIEnv *env, jobject instance,
                                            jlong matAddrGray,              // pointer to grayscale image matrix - input to Apriltags detection
//                                            jlong matAddrRgb,               // pointer to RGB image matrix - input and output
                                            jlong tagDetectorPointer,       // pointer to an AprilTags::TagDetector instance
                                            jlong tagFamilyPointer,
                                            jdouble tagSize_metres,
                                            jdouble fx_pixels,
                                            jdouble fy_pixels               ){
    double start_ms = now_ms(); // start time
    double time_then = start_ms;
    double time_now = time_then;
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmich: START %f ms.", (time_now-start_ms));

    apriltag_detector_t *tagDetector = (apriltag_detector_t *)tagDetectorPointer;
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmich: cast tagDetector pointer: %f ms.", (time_now-time_then));

//    LOGI("MainActivity_aprilTagsUmich: tagDetector has %d tag families loaded", tagDetector->tag_families->size );

//    apriltag_detector_clear_families_leave_initialised(tagDetector);
//    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmich: apriltag_detector_clear_families_leave_initialised: %f ms.", (time_now-time_then));

//    LOGI("MainActivity_aprilTagsUmich: tagDetector has %d tag families loaded", tagDetector->tag_families->size );

    apriltag_family_t *tagFamily = (apriltag_family_t *)tagFamilyPointer;
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmich: cast tagFamily pointer: %f ms.", (time_now-time_then));
    apriltag_detector_add_family(tagDetector, tagFamily);
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmich: apriltag_detector_add_family: %f ms.", (time_now-time_then));

    LOGI("MainActivity_aprilTagsUmich: tagDetector has %d tag families loaded", tagDetector->tag_families->size );


    Mat &mGr = *(Mat *) matAddrGray;
    int width = mGr.cols;       // image size in pixels
    int height = mGr.rows;      // image size in pixels
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmich: Mat &mGr = *(Mat *) matAddrGray: %f ms.", (time_now-time_then));

//    int quiet = getopt_get_bool(getopt, "quiet");
//
//    int maxiters = getopt_get_int(getopt, "iters");

    const int hamm_hist_max = 10;
    int hamm_hist[hamm_hist_max];
    memset(hamm_hist, 0, sizeof(hamm_hist));
    int total_quads = 0;
    int total_hamm_hist[hamm_hist_max];
    memset(total_hamm_hist, 0, sizeof(total_hamm_hist));
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmich: create hamm and hamm_hist_max: %f ms.", (time_now-time_then));

    image_u8_t *im = NULL;
    im = image_u8_create(width, height);
    for (int row_ = 0; row_ < height; row_++) {
        for (int col_ = 0; col_ < width; col_++) {
            im->buf[row_*im->stride + col_ ] = mGr.at<uchar>(row_,col_);  // yth row (from 0), xth pixel (from 0), greyscale so only one channel
        }
    }
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmich: image bytes to image_u8_t: %f ms.", (time_now-time_then));

    zarray_t *detections = apriltag_detector_detect(tagDetector, im);  ////   the meaty bit - run tag detection on a grayscale image given
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmich: do the detections: %f ms.", (time_now-time_then));

    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

            printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, goodness %8.3f, margin %8.3f, centre pixel (%8.3f,%8.3f)\n",
                   i, det->family->d*det->family->d, det->family->h, det->id, det->hamming, det->goodness, det->decision_margin
                    , det->c[0],det->c[1]);
            printf( "\t\tcorners X = [%5.0f, %5.0f, %5.0f, %5.0f]\n" , det->p[0][0], det->p[1][0], det->p[2][0], det->p[3][0] );
            printf( "\t\tcorners y = [%5.0f, %5.0f, %5.0f, %5.0f]\n" , det->p[0][1], det->p[1][1], det->p[2][1], det->p[3][1] );
            printf( " hold on; plot([%5.0f], [%5.0f],'+','LineWidth',2);\n" , det->c[0],det->c[1] );
            printf( " hold on; plot([%5.0f, %5.0f, %5.0f, %5.0f, %5.0f]" , det->p[0][0], det->p[1][0], det->p[2][0], det->p[3][0], det->p[0][0] );
            printf( ", [%5.0f, %5.0f, %5.0f, %5.0f, %5.0f],'--+','LineWidth',2);\n\n" , det->p[0][1], det->p[1][1], det->p[2][1], det->p[3][1], det->p[0][1] );
        hamm_hist[det->hamming]++;
        total_hamm_hist[det->hamming]++;
        time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmich: list tag detection attributes: %f ms  for tag detection %d.", (time_now-time_then), i);
    }

    apriltag_detections_destroy(detections);
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmich: apriltag_detections_destroy: %f ms.", (time_now-time_then));

    total_quads += tagDetector->nquads;

    image_u8_destroy(im);
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmich: image_u8_destroy(im): %f ms.", (time_now-time_then));

    apriltag_detector_destroy(tagDetector);
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmich: apriltag_detector_destroy(tagDetector): %f ms.", (time_now-time_then));

    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmich: tag36h11_destroy(tagFamily): %f ms.", (time_now-time_then));

    return -9000;
    }


jlong JNICALL Java_william_chamberlain_androidvosopencvros_MainActivity_newTagFamilyUmich(JNIEnv *, jobject) {
    double start_ms = now_ms(); // start time
    double time_then = start_ms;
    double time_now = time_then;

    apriltag_family_t *tagFamily = NULL;
    const char *famname = "tag36h11";
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
    else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        LOGI("MainActivity_aprilTagsUmich: Unrecognized tag family name: %s .",famname);
        exit(-1);
    }
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmich: setting up tag family: %f ms.", (time_now-time_then));

    tagFamily->black_border = 1; //getopt_get_int(getopt, "border");
    LOGI("MainActivity_aprilTagsUmich: black_border=%d .", tagFamily->black_border);

    return (long)(tagFamily);
}

jlong JNICALL Java_william_chamberlain_androidvosopencvros_MainActivity_newTagDetectorUmich(JNIEnv *, jobject, jlong tagFamilyPointer) {
    double start_ms = now_ms(); // start time
    double time_then = start_ms;
    double time_now = time_then;

    apriltag_family_t *tagFamily = (apriltag_family_t *)tagFamilyPointer;
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_newTagDetectorUmichh: cast tagFamily pointer: %f ms.", (time_now-time_then));

    //// start from main()

    getopt_t *getopt = getopt_create();
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmich: getopt_create(): %f ms.", (time_now-time_then));

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
    getopt_add_int(getopt, 'i', "iters", "1", "Repeat processing on input set this many times");
    getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input; negative sharpens");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_bool(getopt, '1', "refine-decode", 0, "Spend more time trying to decode tags");
    getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");

    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmich: getopt_add_bool, etc: %f ms.", (time_now-time_then));

//    int argc=0;
//    char *argv[]={"_"};
//    if (!getopt_parse(getopt, argc, argv, 1) || getopt_get_bool(getopt, "help")) {
//        printf("Usage: %s [options] <input files>\n", argv[0]);
//        getopt_do_usage(getopt);
//        exit(0);
//    }

    const zarray_t *inputs = getopt_get_extra_args(getopt);
//
//    apriltag_family_t *tagFamily = NULL;
//    const char *famname = getopt_get_string(getopt, "family");
//    if (!strcmp(famname, "tag36h11"))
//        tagFamily = tag36h11_create();
//    else if (!strcmp(famname, "tag36h10"))
//        tagFamily = tag36h10_create();
//    else if (!strcmp(famname, "tag36artoolkit"))
//        tagFamily = tag36artoolkit_create();
//    else if (!strcmp(famname, "tag25h9"))
//        tagFamily = tag25h9_create();
//    else if (!strcmp(famname, "tag25h7"))
//        tagFamily = tag25h7_create();
//    else {
//        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
//        LOGI("MainActivity_aprilTagsUmich: Unrecognized tag family name: %s .",famname);
//        exit(-1);
//    }
//    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmich: setting up tag family: %f ms.", (time_now-time_then));
//
//    tagFamily->black_border = getopt_get_int(getopt, "border");
//    LOGI("MainActivity_aprilTagsUmich: black_border=%d .", getopt_get_int(getopt, "border"));

//    apriltag_detector_t *tagDetector = createDetector(tagFamily);
    apriltag_detector_t *tagDetector = apriltag_detector_create();              //// DOING TODO - move outside to re-use; same as for Troy's tracker
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_createDetector: created apriltag_detector_t: %f ms.", (time_now-time_then));
    apriltag_detector_add_family(tagDetector, tagFamily);
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_createDetector: added tag family: %f ms.", (time_now-time_then));
    apriltag_detector_add_family(tagDetector, tagFamily);
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_createDetector: added tag family the second time: %f ms.", (time_now-time_then));
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmich: tagDetector: %f ms.", (time_now-time_then));
    tagDetector->quad_decimate = getopt_get_double(getopt, "decimate");
    tagDetector->quad_sigma = getopt_get_double(getopt, "blur");
    tagDetector->nthreads = getopt_get_int(getopt, "threads");
    tagDetector->debug = getopt_get_bool(getopt, "debug");
    tagDetector->refine_edges = getopt_get_bool(getopt, "refine-edges");
    tagDetector->refine_decode = getopt_get_bool(getopt, "refine-decode");
    tagDetector->refine_pose = getopt_get_bool(getopt, "refine-pose");
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmich: tagDetector set attributes: %f ms.", (time_now-time_then));

    apriltag_detector_clear_families_leave_initialised(tagDetector);
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_aprilTagsUmich: apriltag_detector_clear_families_leave_initialised: %f ms.", (time_now-time_then));

    return (long)(tagDetector);
}

void JNICALL Java_william_chamberlain_androidvosopencvros_MainActivity_deleteTagDetectorUmich(JNIEnv *, jobject, jlong tagDetectorPointer, jlong tagFamilyPointer) {
    double start_ms = now_ms(); // start time
    double time_then = start_ms;
    double time_now = time_then;
    apriltag_detector_t *tagDetector = (apriltag_detector_t *)tagDetectorPointer;
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_deleteTagDetectorUmich: cast tagDetector pointer: %f ms.", (time_now-time_then));
    apriltag_detector_clear_families_leave_initialised(tagDetector);
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_deleteTagDetectorUmich: apriltag_detector_clear_families_leave_initialised: %f ms.", (time_now-time_then));
    apriltag_family_t *tagFamily = (apriltag_family_t *)tagFamilyPointer;
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_deleteTagDetectorUmich: cast tagFamily pointer: %f ms.", (time_now-time_then));
    apriltag_detector_add_family(tagDetector, tagFamily);
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_deleteTagDetectorUmich: apriltag_detector_add_family: %f ms.", (time_now-time_then));
    apriltag_detector_destroy( (apriltag_detector_t *)tagDetectorPointer );
    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_deleteTagDetectorUmich: apriltag_detector_destroy: %f ms.", (time_now-time_then));

    //  tag36h11_destroy(tagFamily);        TODO
}

//apriltag_detector_t *createDetector( apriltag_family_t const *tagFamily ) {
//    double start_ms = now_ms(); // start time
//    double time_then = start_ms;
//    double time_now = time_then;
//    apriltag_detector_t *tagDetector = apriltag_detector_create();              //// TODO - move outside to re-use; same as for Troy's tracker
//    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_createDetector: created apriltag_detector_t: %f ms.", (time_now-time_then));
//    apriltag_detector_add_family(tagDetector, tagFamily);
//    time_then = time_now;  time_now = now_ms(); LOGI("MainActivity_createDetector: added tag family: %f ms.", (time_now-time_then));
//    return tagDetector;
//}


}