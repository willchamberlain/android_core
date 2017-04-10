package william.chamberlain.androidvosopencv;

import android.content.Context;
import android.content.pm.PackageManager;
import android.hardware.Camera;
import android.hardware.SensorManager;
import android.hardware.camera2.CameraManager;
import android.location.LocationManager;
import android.os.Bundle;
import android.support.annotation.NonNull;
import android.view.SurfaceView;
import android.view.Window;
import android.widget.Toast;

import org.opencv.core.CvType;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Scalar;

import android.view.WindowManager;

import java.util.Arrays;
import java.util.regex.Matcher;
import java.util.regex.Pattern;


import android.util.Log;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;

import static java.lang.Math.PI;
import static java.lang.Math.tan;

/**
 * @author chadrockey@gmail.com (Chad Rockey)
 * @author axelfurlan@gmail.com (Axel Furlan)
 */


public class MainActivity
        extends         android.app.Activity
        implements      CameraBridgeViewBase.CvCameraViewListener2,
//            PosedEntity,  // Camera has a pose in the world; defaults to aligned with the map coordinate frame origin and axes.
            DimmableScreen, VariableResolution, VisionSource {


    private static final String TAG = "vos_aa1::MainActivity";
    public static final String MARKER_NAMESPACE = "apriltags_marker_publisher/tag_markers";

    private CameraBridgeViewBase _cameraBridgeViewBase;


    private long tagDetectorPointer; // Apriltags

    private Mat matGray;
    private Mat matRgb;
    private Mat mRgbaTransposed;
    private Mat mRgbaFlipped;

    Pattern tagPattern = Pattern.compile("tag ([0-9]+) at x=([0-9-]+\\.[0-9]+) y=([0-9-]+\\.[0-9]+) z=([0-9-]+\\.[0-9]+) roll=([0-9-]+\\.[0-9]+) pitch=([0-9-]+\\.[0-9]+) yaw=([0-9-]+\\.[0-9]+) qx=([0-9-]+\\.[0-9]+) qy=([0-9-]+\\.[0-9]+) qz=([0-9-]+\\.[0-9]+) qw=([0-9-]+\\.[0-9]+)");

    private double[] position    = {0.0,0.0,1.0};     // = new double[3]
    private double[] orientation = {0.0,0.0,0.0,1.0}; // = new double[4]
    private boolean  poseKnown   = false;

    int framesProcessed = 0;
//    PowerManager.WakeLock screenLock;


    private BaseLoaderCallback _baseLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i(TAG, "OpenCV loaded successfully");
                    // Load ndk built module, as specified in moduleName in build.gradle
                    // after opencv initialization
                    System.loadLibrary("native-lib");
                    System.loadLibrary("apriltags_kaess");
                    _cameraBridgeViewBase.enableView();
                }
                break;
                default: {
                    super.onManagerConnected(status);
                }
            }
        }
    };


    private LocationManager mLocationManager;
    private SensorManager mSensorManager;

    private boolean runImageProcessing = false;


    public MainActivity() {
        super();
//        super("VOS AA1", "VOS AA1", URI.create("http://192.168.1.164:11311"));  //TODO // see http://wiki.ros.org/android/Tutorials/indigo/RosActivity , http://answers.ros.org/question/55874/skip-master-chooser/
    }

    CameraManager cameraManager() {
        return (CameraManager) this.getSystemService(CAMERA_SERVICE);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mLocationManager = (LocationManager)this.getSystemService(Context.LOCATION_SERVICE);
        mSensorManager = (SensorManager)this.getSystemService(SENSOR_SERVICE);
        //  cameraManager().getCameraCharacteristics();  -- requires API 21


        // Load ndk built module, as specified
        // in moduleName in build.gradle
        System.loadLibrary("native-lib");
        System.loadLibrary("apriltags_kaess");
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_main);


//      Does not work to turn the screen off programmatically
//      - can set the wake_setting time to 1 second,
//          but that can cause problems if the application is stopped or fails before reseting it to something reasonable like 10 minutes:
//          if the user has to restart they have no time to do anything before the screen turns off
//        screenLock =    ((PowerManager)getSystemService(POWER_SERVICE)).newWakeLock(
//                PowerManager.PARTIAL_WAKE_LOCK, "MainActivity.onCameraFrame");

//        // Permissions for Android 6+
//        ActivityCompat.requestPermissions(MainActivity.this,
//                new String[]{Manifest.permission.CAMERA},
//                1);

        _cameraBridgeViewBase = (CameraBridgeViewBase) findViewById(R.id.main_surface);
        ////  Log.d(TAG, "MainActivity: onCreate: before running _cameraBridgeViewBase.setMaxFrameSize(640,480)");
        _cameraBridgeViewBase.setMaxFrameSize(640,480);  // http://stackoverflow.com/questions/17868954/android-opencv-how-to-set-camera-resolution-when-using-camerabridgeviewbase
        ////  Log.d(TAG, "MainActivity: onCreate: after running _cameraBridgeViewBase.setMaxFrameSize(640,480)");
        _cameraBridgeViewBase.setVisibility(SurfaceView.VISIBLE);
        _cameraBridgeViewBase.setCvCameraViewListener(this);

        if (!OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, _baseLoaderCallback)){  // http://stackoverflow.com/questions/24732504/live-stream-video-processing-in-android-using-opencv
            Log.e("OPENCV", "onCreate: Cannot connect to OpenCV Manager");
        }else {
            Log.i("OPENCV", "onCreate: opencv successfull");
        }

        tagDetectorPointer = newTagDetector();
    }

    @Override
    public void onPause() {
        super.onPause();
        disableCamera();
    }

    @Override
    public void onResume() {
        super.onResume();

//        ////  Log.d(TAG, "MainActivity: onResume: before running AndroidCameraAdapterForDepricatedApi.setCameraToLowestResolution()");
//        AndroidCameraAdapterForDepricatedApi.setCameraToLowestResolution();
//        ////  Log.d(TAG, "MainActivity: onResume: after running AndroidCameraAdapterForDepricatedApi.setCameraToLowestResolution()");

        if (!OpenCVLoader.initDebug()) {
            ////  Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, _baseLoaderCallback);
//            _cameraBridgeViewBase.enableView();
        } else {
            ////  Log.d(TAG, "OpenCV library found inside package. Using it!");
            _baseLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }





    @Override
    public void onRequestPermissionsResult(int requestCode, String permissions[], int[] grantResults) {
        switch (requestCode) {
            case 1: {
                // If request is cancelled, the result arrays are empty.
                if (grantResults.length > 0
                        && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    // permission was granted, yay! Do the
                    // contacts-related task you need to do.
                } else {
                    // permission denied, boo! Disable the
                    // functionality that depends on this permission.
                    Toast.makeText(MainActivity.this, "Permission denied to read your External storage", Toast.LENGTH_SHORT).show();
                }
                return;
            }
            // other 'case' lines to check for other
            // permissions this app might request
        }
    }


    public void onDestroy() {
        super.onDestroy();
        deleteTagDetector(tagDetectorPointer); // Apriltags
        disableCamera();
    }

    public void disableCamera() {
        if (_cameraBridgeViewBase != null) {
            _cameraBridgeViewBase.disableView();
        }
    }

    public void onCameraViewStarted(int width, int height) {
        ////  Log.d(TAG, "MainActivity: onCameraViewStarted("+width+","+height+"): start");
        mRgbaFlipped = new Mat(height,width, CvType.CV_8UC4);
        mRgbaTransposed = new Mat(height,width, CvType.CV_8UC4);
        matRgb = new Mat(height,width, CvType.CV_8UC4);
        ////  Log.d(TAG, "MainActivity: onCameraViewStarted("+width+","+height+"): end");
    }

    public void onCameraViewStopped() {
        if( null != matGray) { matGray.release(); }
        if( null != matRgb) { matRgb.release(); }
        if( null != mRgbaFlipped) { mRgbaFlipped.release(); }
        if( null != mRgbaTransposed) { mRgbaTransposed.release(); }
    }


    boolean screenLocked = false;
    boolean registeredAsVisionSource = false;

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        framesProcessed++;


        // TODO - needs API 21
//        float[] f = characteristics.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS);
//        for (float d : f) {
//            Logger.logGeneral("LENS_INFO_AVAILABLE_FOCAL_LENGTHS : " + d);
//        }

//        Display display = ((WindowManager) this.getSystemService(Context.WINDOW_SERVICE)).getDefaultDisplay();
//        int rotation = display.getRotation();
//        ////  Log.d(TAG, "rotation = " + rotation);

        matGrayFromInput(inputFrame);
        matRgbFromInput(inputFrame);
        ////  Log.d(TAG, "MainActivity: onCameraFrame("+matGray.size().width+","+matGray.size().height+"): start");

//      should be able to use  disableView()
//        if(!runImageProcessing){
//            return matRgb;
//        }

//        getWindow().getContext().getSystemService()
        double focal_length_in_pixels_x;
        double focal_length_in_pixels_y;
        Camera camera = AndroidCameraAdapterForDepricatedApi.getCameraInstance();
//        if(null == camera) {
//            try { camera = _cameraBridgeViewBase.camera(); }
//            catch (Exception e) {
//                ////  Log.d(TAG, "MainActivity: onCameraFrame: exception in camera = _cameraBridgeViewBase.camera() : "+e.getMessage());
//            }
//        }
        
        if(null!=camera) {
            dumm_focalLength_1(camera);
        } else {
            ////  Log.d(TAG, "MainActivity: onCameraFrame: AndroidCameraAdapterForDepricatedApi.getCameraInstance() returns null");
            boolean connected = false;
            for (int camIdx = 0; camIdx < Camera.getNumberOfCameras(); ++camIdx) {      //  see /mnt/nixbig/downloads/chulcher_ros_android_will_fork/android_core/openCVLibrary310/src/main/java/org/opencv/android/JavaCameraView.java:85
                ////  Log.d(TAG, "Trying to open camera with new open(" + Integer.valueOf(camIdx) + ")");
                try {
                    camera = Camera.open(camIdx);
                    connected = true;
                } catch (RuntimeException e) {
                    Log.e(TAG, "Camera #" + camIdx + "failed to open: " + e.getLocalizedMessage());
                }
                if (connected) break;
            }
            if(connected) {
                dumm_focalLength_2(camera);
            } else {
                ////  Log.d(TAG, "MainActivity: onCameraFrame: could not get a camera at all : using the Camera 1 API");
            }
        }

                /*
                See  https://play.google.com/books/reader?id=hb8FCgAAQBAJ&printsec=frontcover&output=reader&hl=en_GB&pg=GBS.PA103.w.12.0.0  pp124-126
                */
                /* Get the focal length in pixels for AprilTags --> position calculation. */
        dumm_focalLength_3();



        focal_length_in_pixels_x = 536.798469;  // TODO - for Samsung Galaxy S3s from /mnt/nixbig/ownCloud/project_AA1__1_1/results/2016_12_04_callibrate_in_ROS/calibrationdata_227/ost.txt
        focal_length_in_pixels_y = 536.571799;  // TODO - for Samsung Galaxy S3s from /mnt/nixbig/ownCloud/project_AA1__1_1/results/2016_12_04_callibrate_in_ROS/calibrationdata_227/ost.txt
//        focal_length_in_pixels_x = 519.902859;  // TODO - for Samsung Galaxy S3s from /mnt/nixbig/ownCloud/project_AA1__1_1/results/2016_12_04_callibrate_in_ROS/calibrationdata_grey/ost.txt
//        focal_length_in_pixels_y = 518.952669;  // TODO - for Samsung Galaxy S3s from /mnt/nixbig/ownCloud/project_AA1__1_1/results/2016_12_04_callibrate_in_ROS/calibrationdata_grey/ost.txt
//        Core.flip(matGray,matGray,1);
//        Core.flip(matRgb,matRgb,1);
// TODO - try reducing image size to increase framerate , AND check /Users/will/Downloads/simbaforrest/cv2cg_mini_version_for_apriltag , https://github.com/ikkiChung/MyRealTimeImageProcessing , http://include-memory.blogspot.com.au/2015/02/speeding-up-opencv-javacameraview.html , https://developer.qualcomm.com/software/fastcv-sdk , http://nezarobot.blogspot.com.au/2016/03/android-surfacetexture-camera2-opencv.html , https://www.youtube.com/watch?v=nv4MEliij14 ,
        double tagSize_metres = 0.168d;
        String[] tags = getAprilTags(focal_length_in_pixels_x, focal_length_in_pixels_y, tagSize_metres);
//        for(String tag : tags) {
//            ////  Log.d(TAG, "-------------------------------------------------------");
//            ////  Log.d(TAG, "---");
//            System.out.print(tag);
//            ////  Log.d(TAG, "---");
//        }
        ////  Log.d(TAG, "---------- detected " + tags.length + " tags ----------------------------------------------------------------------------------------------------");


        for(String tag : tags) {
            {
                ////  Log.d(TAG, "-------------------------------------------------------");
                ////  Log.d(TAG, "---: "); System.out.print(tag); ////  Log.d(TAG, " :---");
//                ////  Log.d(TAG, "   checking for pattern  [[" + tagPattern.toString() + "]]");
//                ////  Log.d(TAG, "   ... in string [[" + tag + "]]");
            }
            Matcher matcher = dumm_matchTagString(tag);
            {
                ////  Log.d(TAG, "--- matcher matches regex in the string? : "); System.out.println(matcher.matches());
            }
            String tagId = dumm_pullGroup1fromMatcher(matcher, 1);
            dumm_parseIntToInt(tagId);
            {
                ////  Log.d(TAG, "--- matched tag_id="); System.out.print(tagId); ////  Log.d(TAG, ", matched x=");  System.out.print(dumm_pullGroup1fromMatcher(matcher, 2)); ////  Log.d(TAG, "---");
//                ////  Log.d(TAG, "-------------------------------------------------------");
            }
//            if(null != aprilTagsPosePublisher) {aprilTagsPosePublisher.publishAprilTagId(Integer.parseInt(tagId));}
//            else { ////  Log.d(TAG, "MainActivity: onCameraFrame: aprilTagsPosePublisher is null: cannot publish tag id "); System.out.println(tagId); }

// ("tag ([0-9]+) at x=([0-9-]+\\.[0-9]+) y=([0-9-]+\\.[0-9]+) z=([0-9-]+\\.[0-9]+) roll=([0-9-]+\\.[0-9]+) pitch=([0-9-]+\\.[0-9]+) yaw=([0-9-]+\\.[0-9]+) qx=([0-9-]+\\.[0-9]+) qy=([0-9-]+\\.[0-9]+) qz=([0-9-]+\\.[0-9]+) qw=([0-9-]+\\.[0-9]+)");
            dumm_parseDoubles(matcher);

//                ////  Log.d(TAG, "--- detectedFeaturesClient.reportDetectedFeature --- ");
//                ////  Log.d(TAG, "--- tag_id=");System.out.print(tagId);
//                    ////  Log.d(TAG, " :  x=");System.out.print(matcher.group(2));////  Log.d(TAG, " y=");System.out.print(matcher.group(3));////  Log.d(TAG, " z=");System.out.print(matcher.group(4));
//                    ////  Log.d(TAG, " :  roll=");System.out.print(matcher.group(5));////  Log.d(TAG, " pitch=");System.out.print(matcher.group(6));////  Log.d(TAG, " yaw=");System.out.print(matcher.group(7));
//                    ////  Log.d(TAG, " :  qx=");System.out.print(matcher.group(8));////  Log.d(TAG, " qy=");System.out.print(matcher.group(9));////  Log.d(TAG, " qz=");System.out.print(matcher.group(10));////  Log.d(TAG, " qw=");System.out.print(matcher.group(11));
//                    ////  Log.d(TAG, "---");
//                ////  Log.d(TAG, "-------------------------------------------------------");
//                {
//                    //markerPublisherNode.publishMarker(String marker_namespace_, int marker_id_, String marker_text_, double x,double y,double z,double qx,double qy,double qz,double qw, String parent_frame_id, Time time_) {
//                    markerPublisherNode.publishMarker(MARKER_NAMESPACE, tagId_int, tagId, x, y, z, qx, qy, qz, qw, Naming.cameraFrameId(getCamNum()), timeNow);
//                    // TODO - use same variable for this and aa1_vos_android_catkin_ws___src/vos_aa1/src/vos_aa1/detect_feature_server.py
//                    // TODO - publish markers in detected_feature_server.py - once I have figured out what is going on with the RPY in that Python code
//                    // TODO -   ... or use C++ as detected_feature_server.cpp
//                }
        }


//        mRgbaTransposed = matRgb.t();
//        Imgproc.resize(mRgbaTransposed, mRgbaFlipped, matRgb.size(),0,0,0);
//        Core.flip(mRgbaFlipped, matRgb, 1); // see - http://answers.opencv.org/question/20325/how-can-i-change-orientation-without-ruin-camera-settings/

        if (screenLocked) {
            ////  Log.d(TAG, "onCameraFrame: screenLocked = true at frame "+framesProcessed+": setting output matrices to black");
            dumm_turnOutputBlack();
        }
        return matRgb;
//
//        if(Surface.ROTATION_0==rotation) {
//            salt(matGray.getNativeObjAddr(), 10000);
//        } else if (Surface.ROTATION_90==rotation) {
//            canny(matGray.getNativeObjAddr());
//        }else if (Surface.ROTATION_180==rotation) {
//            salt(matGray.getNativeObjAddr(), 2000);
//        }else if (Surface.ROTATION_270==rotation) {
//            salt(matGray.getNativeObjAddr(), 3000);
//        }
//        return matGray;
    }

    private void dumm_focalLength_1(Camera camera) {
        float focalLengthInMetres = camera.getParameters().getFocalLength();
        ////  Log.d(TAG, "MainActivity: onCameraFrame: focalLengthInMetres="+focalLengthInMetres);
        float horizontalAngleView = camera.getParameters().getHorizontalViewAngle();
        ////  Log.d(TAG, "MainActivity: onCameraFrame: horizontalAngleView="+horizontalAngleView);
        double focal_length_in_pixels = (camera.getParameters().getPictureSize().width * 0.5) / tan(horizontalAngleView * 0.5 * PI/180.0);
        ////  Log.d(TAG, "MainActivity: onCameraFrame: focal_length_in_pixels="+focal_length_in_pixels);
    }

    private void dumm_focalLength_2(Camera camera) {
        double focal_length_in_pixels_x;
        double focal_length_in_pixels_y;
        float focalLengthInMetres = camera.getParameters().getFocalLength();
        ////  Log.d(TAG, "MainActivity: onCameraFrame: focalLengthInMetres="+focalLengthInMetres);
        float horizontalAngleView = camera.getParameters().getHorizontalViewAngle();
        ////  Log.d(TAG, "MainActivity: onCameraFrame: horizontalAngleView="+horizontalAngleView);
        focal_length_in_pixels_x = (camera.getParameters().getPictureSize().width * 0.5) / tan(horizontalAngleView * 0.5 * PI/180.0);
        ////  Log.d(TAG, "MainActivity: onCameraFrame: focal_length_in_pixels_x="+focal_length_in_pixels_x);
        float verticalAngleView = camera.getParameters().getVerticalViewAngle();
        ////  Log.d(TAG, "MainActivity: onCameraFrame: getVerticalViewAngle()="+verticalAngleView);
        focal_length_in_pixels_y = (camera.getParameters().getPictureSize().width * 0.5) / tan(verticalAngleView* 0.5 * PI/180.0);
        ////  Log.d(TAG, "MainActivity: onCameraFrame: focal_length_in_pixels_y="+focal_length_in_pixels_y);
    }

    private void dumm_focalLength_3() {
        double focal_length_in_pixels_x;
        double focal_length_in_pixels_y;
        float[] estimatedFocusedDistances = {9000.0f,9000.0f,9000.0f};      // dummy values, overridden in cameraParameters().getFocusDistances(estimatedFocusedDistances)
        cameraParameters().getFocusDistances(estimatedFocusedDistances);    // focus distances in meters. the distances from the camera to where an object appears to be in focus. The object is sharpest at the optimal focus distance. The depth of field is the far focus distance minus near focus distance.param 'output' must be a float array with three elements. Near focus distance, optimal focus distance, and far focus distance will be filled in the array
        ////  Log.d(TAG, "MainActivity: onCameraFrame: estimatedFocusedDistances = "+ Arrays.toString(estimatedFocusedDistances));
        MatOfDouble mProjectionCV = new MatOfDouble();
        mProjectionCV.create(3,3, CvType.CV_64FC1);
        final double fovAspectRatio = fieldOfViewX() / fieldOfViewY();
        double diagonalPixels = Math.sqrt( (Math.pow(matGray.size().width, 2.0)) + (Math.pow(matGray.size().width/fovAspectRatio, 2.0)) );
        double diagonalFov = Math.sqrt( (Math.pow(fieldOfViewX(), 2.0)) + (Math.pow(fieldOfViewY(), 2.0)) );
        double focalLengthPixels = diagonalPixels / (2.0 * Math.tan(0.5 * diagonalFov * Math.PI/180.0f ));
        focal_length_in_pixels_x = ( matGray.size().width / (2.0 * Math.tan(0.5 * fieldOfViewX() * Math.PI/180.0f)) );
        focal_length_in_pixels_y = ( matGray.size().height / (2.0 * Math.tan(0.5 * fieldOfViewY() * Math.PI/180.0f)) );
        ////  Log.d(TAG, "MainActivity: onCameraFrame: "
//                +"  OpenCV matrix: width pixels="+matGray.size().width+", height pixels="+matGray.size().height
//                +", preview: width pixels="+cameraParameters().getPreviewSize().width+", height pixels="+cameraParameters().getPreviewSize().height
//                +", zoom value="+cameraParameters().getZoom()+", zoom as percentage="+cameraParameters().getZoomRatios().get(cameraParameters().getZoom()).intValue()
//                +", fieldOfView: X rads="+fieldOfViewX()+", Y rads="+fieldOfViewY()
//                +", fovAspectRatio="+fovAspectRatio+", diagonalPixels="+diagonalPixels
//                +", diagonalFov rads="+diagonalFov+", focalLengthPixels - diagonal="+focalLengthPixels);
        ////  Log.d(TAG, "MainActivity: onCameraFrame: "
//                +", focal length x = "+  focal_length_in_pixels_x
//                +", focal length y = "+  focal_length_in_pixels_y );
    }

    private void dumm_parseIntToInt(String tagId) {
        Integer tagId_integer = Integer.parseInt(tagId);
        int tagId_int = tagId_integer.intValue();
    }

    private void dumm_turnOutputBlack() {
        Scalar blackScalar = new Scalar(0); //,CvType.CV_8UC4
        matRgb.setTo(blackScalar);
    }

    private String dumm_pullGroup1fromMatcher(Matcher matcher, int group) {
        return matcher.group(group);
    }

    @NonNull
    private Matcher dumm_matchTagString(String tag) {
        return tagPattern.matcher(tag);
    }

    private void dumm_parseDoubles(Matcher matcher) {
        double x = Double.parseDouble(matcher.group(2));
        double y = Double.parseDouble(matcher.group(3));
        double z = Double.parseDouble(matcher.group(4));
        double roll  = Double.parseDouble(matcher.group(5));
        double pitch = Double.parseDouble(matcher.group(6));
        double yaw   = Double.parseDouble(matcher.group(7));
        double qx = Double.parseDouble(matcher.group(8));
        double qy = Double.parseDouble(matcher.group(9));
        double qz = Double.parseDouble(matcher.group(10));
        double qw = Double.parseDouble(matcher.group(11));
    }

    private String[] getAprilTags(double focal_length_in_pixels_x, double focal_length_in_pixels_y, double tagSize_metres) {
        return aprilTags(matGray.getNativeObjAddr(),matRgb.getNativeObjAddr(),tagDetectorPointer, tagSize_metres, focal_length_in_pixels_x, focal_length_in_pixels_y);
    }

    private void matRgbFromInput(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        matRgb  = inputFrame.rgba();
    }

    private void matGrayFromInput(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        matGray = inputFrame.gray();
    }

    @Override
    public void screenOff() {
        screenLocked = true;

        final WindowManager.LayoutParams params = this.getWindow().getAttributes();
        params.flags = WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON;
        //TODO Store original brightness value
        params.screenBrightness = 0.1f;
        final Window w = this.getWindow();
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                w.setAttributes(params);
            }
        });
    }

    @Override
    public void screenOn() {
        //TODO restore from original brightness value
        screenOn(1.0f);
    }

    public void screenOn(float percentBrightness) {
        screenLocked = false;
        if(0.0 > percentBrightness) {
            percentBrightness = 0.0f;
        } else if (1.0 < percentBrightness) {
            percentBrightness = 1.0f;
        }
        // --> android.view.ViewRootImpl$CalledFromWrongThreadException: Only the original thread that created a view hierarchy can touch its views.
        final WindowManager.LayoutParams params = this.getWindow().getAttributes();
        params.flags = WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON;
        params.screenBrightness = percentBrightness;
        final Window w = this.getWindow();
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                w.setAttributes(params);
            }
        });
    }


    public double[] getPosition() {
        return position;
    }

    public double[] getOrientation() {
        return orientation;
    }

    public boolean poseKnown(){
        return poseKnown;
    }


    public native void salt(long matAddrGray, int nbrElem);
    public native void canny(long matAddrGray);

    public native long newTagDetector();   // Apriltags
    public native void deleteTagDetector(long tagDetectorPointer);     // Apriltags
    public native String[] aprilTags(long matAddrGray, long matAddrRgb, long tagDetectorPointer, double tagSize_metres, double fx_pixels, double fy_pixels);  // Apriltags


    @Override
    public void lowResolution() {
        AndroidCameraAdapterForDepricatedApi.setCameraToLowestResolution();
    }

    @Override
    public void highResolution() {
        AndroidCameraAdapterForDepricatedApi.setCameraToHighestResolution();
    }

    public void resolutionMax(final int width, final int height) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                _cameraBridgeViewBase.setMaxFrameSize(width, height);
                _cameraBridgeViewBase.disableView();
                _cameraBridgeViewBase.enableView();
            }
        });
    }

    public void resolutionMin(final int width, final int height) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
            _cameraBridgeViewBase.setMinimumWidth(width);
            _cameraBridgeViewBase.setMinimumHeight(height);
            _cameraBridgeViewBase.disableView();
            _cameraBridgeViewBase.enableView();
            }
        });
    }

    public void resolutionMinMax(final int minWidth, final int minHeight, final int maxWidth, final int maxHeight) { // TODO - final ?
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                _cameraBridgeViewBase.setMinimumWidth(minWidth);
                _cameraBridgeViewBase.setMinimumHeight(minHeight);
                _cameraBridgeViewBase.setMaxFrameSize(maxWidth, maxHeight);
                _cameraBridgeViewBase.disableView();
                _cameraBridgeViewBase.enableView();
            }
        });
    }

    @Override
    public void start() {
        runImageProcessing = true;
        _cameraBridgeViewBase.enableView();
    }

    @Override
    public void stop() {
        runImageProcessing = false;
        _cameraBridgeViewBase.disableView();
    }

    @Override
    public void relocalise() {
        poseKnown = false;
        position    = new double[]{0.0,0.0,1.0};
        orientation = new double[]{0.0,0.0,0.0,1.0};
    }

    public void publishCurrentFrame() {

    }


    /*
    See http://stackoverflow.com/questions/3261776/determine-angle-of-view-of-smartphone-camera/12118760#12118760
     */
    private static double zoomAngle(double degrees, int zoom) {
        double theta = Math.toRadians(degrees);
        return 2d * Math.atan(100d * Math.tan(theta / 2d) / zoom);
    }


    /*
    See http://stackoverflow.com/questions/3261776/determine-angle-of-view-of-smartphone-camera/12118760#12118760
     */
    public void calculateViewAngles() {
        Camera.Parameters p = _cameraBridgeViewBase.camera().getParameters();
        int zoom = p.getZoomRatios().get(p.getZoom()).intValue();
        Camera.Size sz = p.getPreviewSize();
        double aspect = (double) sz.width / (double) sz.height;
        double thetaV = Math.toRadians(p.getVerticalViewAngle());
        double thetaH = 2d * Math.atan(aspect * Math.tan(thetaV / 2));
        thetaV = 2d * Math.atan(100d * Math.tan(thetaV / 2d) / zoom);
        thetaH = 2d * Math.atan(100d * Math.tan(thetaH / 2d) / zoom);
    }

    public double fieldOfViewX() {
        Camera.Parameters p = _cameraBridgeViewBase.camera().getParameters();
        int zoom = p.getZoomRatios().get(p.getZoom()).intValue();
        Camera.Size sz = p.getPreviewSize();
        double aspect = (double) sz.width / (double) sz.height;
        double thetaV = Math.toRadians(p.getVerticalViewAngle());
        double thetaH = 2d * Math.atan(aspect * Math.tan(thetaV / 2));
        thetaH = 2d * Math.atan(100d * Math.tan(thetaH / 2d) / zoom);
        return thetaH;
    }

    public double fieldOfViewY() {
        Camera.Parameters p = _cameraBridgeViewBase.camera().getParameters();
        int zoom = p.getZoomRatios().get(p.getZoom()).intValue();
        Camera.Size sz = p.getPreviewSize();
        double aspect = (double) sz.width / (double) sz.height;
        double thetaV = Math.toRadians(p.getVerticalViewAngle());
        thetaV = 2d * Math.atan(100d * Math.tan(thetaV / 2d) / zoom);
        return thetaV;
    }

    public Camera.Parameters cameraParameters() {
        return _cameraBridgeViewBase.camera().getParameters();
    }

}

