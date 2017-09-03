/**
 Copyright (c) 2017, William Chamberlain, ARC Centre of Excellence for Robotic Vision (ACRV: http://roboticvision.org) - Queensland University of Technology (QUT : http://qut.edu.au)
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, the above attributions of authorship and contribution, this list of conditions, and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, the above attributions of authorship and contribution, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package william.chamberlain.androidvosopencvros;

import android.Manifest;
import android.app.Activity;
import android.app.AlertDialog;
import android.content.ActivityNotFoundException;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.hardware.Camera;
import android.hardware.SensorManager;
import android.location.LocationManager;
import android.net.Uri;
import android.os.Bundle;
import android.support.v4.app.ActivityCompat;
import android.util.Log;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.SurfaceView;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Toast;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Scalar;
import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.regex.Matcher;

import boofcv.abst.fiducial.FiducialDetector;
import boofcv.alg.distort.LensDistortionNarrowFOV;
import boofcv.alg.distort.pinhole.LensDistortionPinhole;
import boofcv.alg.misc.GImageMiscOps;
import boofcv.android.gui.VideoProcessing;
import boofcv.core.encoding.ConvertNV21;
import boofcv.factory.fiducial.ConfigFiducialBinary;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.factory.filter.binary.ConfigThreshold;
import boofcv.factory.filter.binary.ThresholdType;
import boofcv.struct.calib.CameraPinhole;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.ImageGray;
import boofcv.struct.image.ImageType;
import geometry_msgs.Pose;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.EulerType;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Quaternion_F64;
import sensor_msgs.Imu;
import vos_aa1.WhereIsAsPub;
import william.chamberlain.androidvosopencvros.android_mechanics.PermissionsChecker;
import william.chamberlain.androidvosopencvros.device.DimmableScreen;
import william.chamberlain.androidvosopencvros.device.ImuCallback;
import william.chamberlain.androidvosopencvros.monitoring.ImuMonitoringPublisher;
import william.chamberlain.androidvosopencvros.resilient.ResilientNetworkActivity;

import static boofcv.struct.image.ImageDataType.F32;
import static boofcv.struct.image.ImageType.Family.GRAY;
import static java.lang.Math.PI;
import static java.lang.Math.tan;
import static org.opencv.android.CameraBridgeViewBase.CAMERA_ID_BACK;
import static william.chamberlain.androidvosopencvros.Constants.APRIL_TAGS_KAESS_36_H_11;
import static william.chamberlain.androidvosopencvros.Constants.tagSize_metres;
import static william.chamberlain.androidvosopencvros.DataExchange.tagPattern_trans_quat;

/**
 * @author chadrockey@gmail.com (Chad Rockey)
 * @author axelfurlan@gmail.com (Axel Furlan)
 */


public class MainActivityRosLess extends Activity
        implements      CameraBridgeViewBase.CvCameraViewListener2,
        ActivityCompat.OnRequestPermissionsResultCallback,  // to deal with runtime permissions checks - from API 23 oward
            PosedEntity,                                    // Camera has a pose in the world; defaults to aligned with the map coordinate frame origin and axes.
            DetectedFeaturesHolder,
        ImuCallback,
        DimmableScreen, VariableResolution, VisionSource {

    HashMap<String,Boolean> allocatedTargets = new HashMap<String,Boolean>();

    public static final int CAMERA_FRONT_OR_BACK_INIT = CAMERA_ID_BACK;
    private static final boolean running_native = false;


    private static final String TAG = "vos_aa1::MainActivity";
    public static final String MARKER_NAMESPACE = "apriltags_marker_publisher/tag_markers";

    // TODO - try native camera:  see https://stackoverflow.com/questions/16626343/what-is-the-difference-between-opencv-android-javacameraview-and-opencv-andro/28130605#28130605
    private CameraBridgeViewBase _cameraBridgeViewBase;


    private long tagDetectorPointer; // Apriltags

    private Mat matGray;
    private Mat matRgb;
//    private Mat mRgbaTransposed;      // rotate image as preview rotates with device
//    private Mat mRgbaFlipped;         // rotate image as preview rotates with device


    private double[] position    = {0.0,0.0,1.0};     // = new double[3]
    private double[] orientation = {0.0,0.0,0.0,1.0}; // = new double[4]
    private boolean  poseKnown   = false;

    long framesProcessed = 0;
    long frameNumber = 0;
//    PowerManager.WakeLock screenLock;


    private BaseLoaderCallback _baseLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i(TAG, "OpenCV loaded successfully");
                    if(running_native) {
                        // Load ndk built module, as specified in moduleName in build.gradle
                        // after opencv initialization
                        System.loadLibrary("native-lib");
                        System.loadLibrary("apriltags_kaess");
                        System.loadLibrary("apriltags_umich");
                    }
                    _cameraBridgeViewBase.enableView();
                }
                break;
                default: {
                    super.onManagerConnected(status);
                }
            }
        }
    };



    private boolean runImageProcessing = false;
    private boolean displayRgb = true;

    FeatureDataRecorderModeller featureDataRecorderModeller = new FeatureDataRecorderModeller();
    public static final int MIN_FRAME_SIZE_WIDTH_INIT = 100;
    public static final int MIN_FRAME_SIZE_HEIGHT_INIT = 100;
    public static final int MAX_FRAME_SIZE_WIDTH_INIT = 400;
    public static final int MAX_FRAME_SIZE_HEIGHT_INIT = 300;


    public MainActivityRosLess() {
        super();
//        super("VOS AA1", "VOS AA1", URI.create("http://192.168.1.164:11311"));  //TODO // see http://wiki.ros.org/android/Tutorials/indigo/RosActivity , http://answers.ros.org/question/55874/skip-master-chooser/
    }

//    CameraManager cameraManager() {
//        return (CameraManager) this.getSystemService(CAMERA_SERVICE);
//    }

    @Override
    protected void onCreate(Bundle savedInstanceState)   // TODO - look at lifecycle doco and do this properly
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        checkPermissions();

//        mLocationManager = (LocationManager)this.getSystemService(Context.LOCATION_SERVICE);
//        mSensorManager = (SensorManager)this.getSystemService(SENSOR_SERVICE);
        //  cameraManager().getCameraCharacteristics();  -- requires API 21


        if( running_native) {
            // Load ndk built module, as specified
            // in moduleName in build.gradle
            System.loadLibrary("native-lib");
            System.loadLibrary("apriltags_kaess");
            System.loadLibrary("apriltags_umich");
        }
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_main);

//      Does not work to turn the screen off programmatically
//      - can set the wake_setting time to 1 second,
//          but that can cause problems if the application is stopped or fails before reseting it to something reasonable like 10 minutes:
//          if the user has to restart they have no time to do anything before the screen turns off
//        screenLock =    ((PowerManager)getSystemService(POWER_SERVICE)).newWakeLock(
//                PowerManager.PARTIAL_WAKE_LOCK, "MainActivityRosLess.onCameraFrame");

//        // Permissions for Android 6+
//        ActivityCompat.requestPermissions(MainActivity.this,
//                new String[]{Manifest.permission.CAMERA},
//                1);

        _cameraBridgeViewBase = (CameraBridgeViewBase) findViewById(R.id.main_surface);
        _cameraBridgeViewBase.setCameraIndex(CAMERA_FRONT_OR_BACK_INIT);  // front-facing camera is the user-side camera i.e. facing toward the front of the device from the user point of view and away from the front of the device from the device point of view, back-facing/rear-facing is usually the primary camera facing away from the user i.e. facing away from the rear of the device
        System.out.println("MainActivityRosLess: onCreate: after running _cameraBridgeViewBase.setCameraIndex("+CAMERA_FRONT_OR_BACK_INIT+")");
        System.out.println("MainActivityRosLess: onCreate: before running _cameraBridgeViewBase.setMaxFrameSize("+ MAX_FRAME_SIZE_WIDTH_INIT +","+ MAX_FRAME_SIZE_HEIGHT_INIT +")");
        _cameraBridgeViewBase.setMaxFrameSize(MAX_FRAME_SIZE_WIDTH_INIT, MAX_FRAME_SIZE_HEIGHT_INIT);  // http://stackoverflow.com/questions/17868954/android-opencv-how-to-set-camera-resolution-when-using-camerabridgeviewbase
        System.out.println("MainActivityRosLess: onCreate: after running _cameraBridgeViewBase.setMaxFrameSize("+ MAX_FRAME_SIZE_WIDTH_INIT +","+ MAX_FRAME_SIZE_HEIGHT_INIT +")");
        _cameraBridgeViewBase.setVisibility(SurfaceView.VISIBLE);
        _cameraBridgeViewBase.setCvCameraViewListener(this);

        resolutionMinMax(MIN_FRAME_SIZE_WIDTH_INIT,MIN_FRAME_SIZE_HEIGHT_INIT,MAX_FRAME_SIZE_WIDTH_INIT,MAX_FRAME_SIZE_HEIGHT_INIT);
        Log.i("onCreate", "onCreate: set resolution low");

        if (!OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, _baseLoaderCallback)){  // http://stackoverflow.com/questions/24732504/live-stream-video-processing-in-android-using-opencv
            Log.e("onCreate", "onCreate: Cannot connect to OpenCV Manager");
        }else {
            Log.i("onCreate", "onCreate: opencv successfull");
        }

        if(running_native) {
//            tagDetectorPointer = newTagDetectorUmich();
//        tagDetectorPointer = newTagDetectorKaess();
            Log.i("onCreate", "tagDetectorPointer created");
        }
    }

    @Override
    public void onPause() {  // TODO - look at lifecycle doco and do this properly
        super.onPause();
        disableCamera();
    }

    @Override
    public void onResume() {  // TODO - look at lifecycle doco and do this properly
        super.onResume();

        checkPermissions();


        allocatedTargets.put(ROBOT_ALLOCATION_KEY,true);
        allocatedTargets.put(TARGET_ALLOCATION_KEY,true);
//        System.out.println("MainActivityRosLess: onResume: before running AndroidCameraAdapterForDepricatedApi.setCameraToLowestResolution()");
//        AndroidCameraAdapterForDepricatedApi.setCameraToLowestResolution();
//        System.out.println("MainActivityRosLess: onResume: after running AndroidCameraAdapterForDepricatedApi.setCameraToLowestResolution()");

        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, _baseLoaderCallback);
//            _cameraBridgeViewBase.enableView();
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            _baseLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }




    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        switch (requestCode) {
            case PermissionsChecker.REQUEST_CODE_ASK_PERMISSIONS:
                // If request is cancelled, the result arrays are empty.
                if (grantResults.length > 0
                        && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    // Permission Granted
                    Log.i("onRequestPermissionsRes", "onRequestPermissionsResult: permissions granted");
                } else {
                    // Permission Denied
                    Toast.makeText(this, permissions[0]+" denied", Toast.LENGTH_SHORT).show();
                }
                break;
            // other 'case' lines to check for other
            // permissions this app might request
            default:
                super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        }
    }



    public void onDestroy() {
        super.onDestroy();
//        deleteTagDetectorUmichOneShot(tagDetectorPointer); // Apriltags
//        deleteTagDetectorKaess(tagDetectorPointer);
        disableCamera();
    }

    public void disableCamera() {
        if (_cameraBridgeViewBase != null) {
            _cameraBridgeViewBase.disableView();
        }
    }

    public void onCameraViewStarted(int width, int height) {
        System.out.println("MainActivityRosLess: onCameraViewStarted("+width+","+height+"): start");
//        mRgbaFlipped = new Mat(height,width, CvType.CV_8UC4);
//        mRgbaTransposed = new Mat(height,width, CvType.CV_8UC4);
        matRgb = new Mat(height,width, CvType.CV_8UC4);
        System.out.println("MainActivityRosLess: onCameraViewStarted("+width+","+height+"): end");
    }

    public void onCameraViewStopped() {
        if( null != matGray) { matGray.release(); }
        if( null != matRgb) { matRgb.release(); }
//        if( null != mRgbaFlipped) { mRgbaFlipped.release(); }
//        if( null != mRgbaTransposed) { mRgbaTransposed.release(); }
    }


    ArrayList<VisionTask> taskQueue = new ArrayList<VisionTask>();

    public void dealWithRequestForInformation(WhereIsAsPub message){
        Log.i(TAG,"dealWithRequestForInformation(WhereIsAsPub message) : "+message.getAlgorithm()+", "+message.getDescriptor()+", "+message.getRequestId()+", "+message.toString() );
        synchronized (this) {
            addVisionTaskToQueue(message);
        }
    }

    private void addVisionTaskToQueue(WhereIsAsPub message) {
        taskQueue.add(new VisionTask()
                .algorithm(message.getAlgorithm())
                .descriptor(message.getDescriptor())
                .requestId(message.getRequestId())
                .relationToBase(message.getRelationToBase())
                .returnUrl(message.getReturnUrl())
                .executionIterations(message.getRate()));
    }

    private void addVisionTaskToQueue(WhereIsAsPubLocal message) {
        taskQueue.add(new VisionTask()
                .algorithm(message.getAlgorithm())
                .descriptor(message.getDescriptor())
                .requestId(message.getRequestId())
                .relationToBase(message.getRelationToBase())
                .returnUrl(message.getReturnUrl())
                .executionIterations(message.getRate()));
    }


    boolean screenLocked = false;
    boolean registeredAsVisionSource = false;
////    List<DetectedFeature> detectedFeatures = Collections.synchronizedList(new ArrayList<DetectedFeature>());
    // avoid synchronising on the list with e.g.   synchronized (detectedFeatures) { for(detectedFeature: detectedFeatures) {...} }
    // TODO - see http://www.codejava.net/java-core/collections/understanding-collections-and-thread-safety-in-java , https://docs.oracle.com/javase/7/docs/api/java/util/concurrent/CopyOnWriteArrayList.html
    List<DetectedFeature> detectedFeatures = new java.util.concurrent.CopyOnWriteArrayList<DetectedFeature>();


boolean readyToProcessImages = true;
String logTag = "MainActivityRosLess";

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        /* Dev: part of robot visual model */
        HashMap<RobotId, List<DetectedTag>> robots = new HashMap<RobotId,List<DetectedTag>>();
        RobotId singleDummyRobotId = new RobotId(555); // new RobotId("dummy robot id");
        List<DetectedTag> robotFeatures = new ArrayList<DetectedTag>();
        /* end Dev: part of robot visual model */
        frameNumber++;
        Log.i(TAG,"onCameraFrame: START: : frame="+frameNumber);
        if(!readyToProcessImages) {
            Log.i(TAG,"onCameraFrame: readyToProcessImages is false: returning image without processing.");
            return inputFrame.gray();
        }
        framesProcessed++;
//// TODO - timing here  c[camera_num]-f[frameprocessed]
Log.i(logTag,"start frame");


        // TODO - needs API 21
//        float[] f = characteristics.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS);
//        for (float d : f) {
//            Logger.logGeneral("LENS_INFO_AVAILABLE_FOCAL_LENGTHS : " + d);
//        }

//        Display display = ((WindowManager) this.getSystemService(Context.WINDOW_SERVICE)).getDefaultDisplay();
//        int rotation = display.getRotation();
//        Log.d(TAG, "rotation = " + rotation);

        matGray = inputFrame.gray();
        matRgb  = inputFrame.rgba();
        System.out.println("MainActivityRosLess: onCameraFrame("+matGray.size().width+","+matGray.size().height+"): start");

//      should be able to use  disableView()
//        if(!runImageProcessing){
//            return matRgb;
//        }

//        getWindow().getContext().getSystemService()
        float focal_length_in_pixels_x;
        float focal_length_in_pixels_y;
        Camera camera = AndroidCameraAdapterForDepricatedApi.getCameraInstance();
//        if(null == camera) {
//            try { camera = _cameraBridgeViewBase.camera(); }
//            catch (Exception e) {
//                System.out.println("MainActivityRosLess: onCameraFrame: exception in camera = _cameraBridgeViewBase.camera() : "+e.getMessage());
//            }
//        }


//        calculateFocalLength_a(camera);     // try calculating the focal length
//        calculateFocalLength_b();             // try calculating the focal length

        // TODO - 640 is now a magic number : it is the image width in pixels at the time of calibration of focal length
        focal_length_in_pixels_x = 519.902859f * ((float)matGray.size().width/640.0f);  // TODO - for Samsung Galaxy S3s from /mnt/nixbig/ownCloud/project_AA1__1_1/results/2016_12_04_callibrate_in_ROS/calibrationdata_grey/ost.txt
        focal_length_in_pixels_y = 518.952669f * ((float)matGray.size().height/480.0f);  // TODO - for Samsung Galaxy S3s from /mnt/nixbig/ownCloud/project_AA1__1_1/results/2016_12_04_callibrate_in_ROS/calibrationdata_grey/ost.txt
//        Core.flip(matGray,matGray,1);
//        Core.flip(matRgb,matRgb,1);
// TODO - try reducing image size to increase framerate , AND check /Users/will/Downloads/simbaforrest/cv2cg_mini_version_for_apriltag , https://github.com/ikkiChung/MyRealTimeImageProcessing , http://include-memory.blogspot.com.au/2015/02/speeding-up-opencv-javacameraview.html , https://developer.qualcomm.com/software/fastcv-sdk , http://nezarobot.blogspot.com.au/2016/03/android-surfacetexture-camera2-opencv.html , https://www.youtube.com/watch?v=nv4MEliij14 ,

        // start BoofCV
        double BOOFCV_TAG_WIDTH=Hardcoding.BOOFCV_MARKER_SIZE_M; // TODO - list of tags and sizes, and tag-groups and sizes
        byte[] current_image_bytes = last_frame_bytes();
        removeExpiredVisionTasks();
        if(null!=current_image_bytes  && !taskQueue.isEmpty()) {
Log.i(logTag,"start convertPreview(last_frame_bytes(), camera);");
            convertPreview(last_frame_bytes(), camera);
Log.i(logTag,"finished convertPreview(last_frame_bytes(), camera);");
            try {
                FiducialDetector<GrayF32> detector = FactoryFiducial.squareBinary(
                        new ConfigFiducialBinary(BOOFCV_TAG_WIDTH), ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 10), GrayF32.class);  // tag size,  type,  ?'radius'?
                //        detector.setLensDistortion(lensDistortion);

                float  px_pixels = (float)(matGray.size().width/2.0);
                float  py_pixels = (float)(matGray.size().height/2.0);
                double skew = 0.0;


                CameraPinhole pinholeModel = new CameraPinhole(
                        focal_length_in_pixels_x, focal_length_in_pixels_y,
                        skew,
                        px_pixels,py_pixels,
                        new Double(matGray.size().width).intValue(),new Double(matGray.size().height).intValue());
                LensDistortionNarrowFOV pinholeDistort = new LensDistortionPinhole(pinholeModel);
                detector.setLensDistortion(pinholeDistort);  // TODO - do BoofCV calibration - but assume perfect pinhole camera for now

//// TODO - timing here  c[camera_num]-f[frameprocessed]
Log.i(logTag,"start detector.detect(image);");
                detector.detect(image);
//// TODO - timing here  c[camera_num]-f[frameprocessed]
Log.i(logTag,"finished detector.detect(image);");

                Log.i(TAG, "onCameraFrame: found "+detector.totalFound()+" tags via BoofCV");

                // see https://boofcv.org/index.php?title=Example_Fiducial_Square_Image
                Se3_F64 targetToSensor = new Se3_F64();
                Point2D_F64 locationPixel = new Point2D_F64();
                for (int i = 0; i < detector.totalFound(); i++) {
                    // detector.getImageLocation(i, locationPixel);        // pixel location in input image

//// TODO - timing here  c[camera_num]-f[frameprocessed]-i[iteration]
String logTagIteration = logTag+"-i"+i;
Log.i(logTagIteration,"start");
                    int tag_id = -1;
                    if( detector.hasUniqueID() ) {
                        System.out.println("Target ID = " + detector.getId(i));
                        long tag_id_long = detector.getId(i);
                        tag_id = (int)tag_id_long;
                        if ((long)tag_id != tag_id_long) {
                            //throw new IllegalArgumentException(l + " cannot be cast to int without changing its value.");
                            System.out.println(" BoofCV: cannot use tag: tag_id_long '"+tag_id_long+"' cannot be cast to int without changing its value.");
                            continue;
                        }
                    }
//// TODO - timing here  c[camera_num]-f[frameprocessed]-i[iteration]-t[tagid]
                    String logTagTag = logTagIteration+"-t"+tag_id;

                    /* Dev: part of robot visual model */
                    boolean visionTaskToExecute = false;
                    synchronized (this) {
                        for (VisionTask visionTask : taskQueue) {
                            Log.i(logTagTag,"visionTask.getDescriptor() = '"+visionTask.getDescriptor()+"', Integer.toString(tag_id) = '"+Integer.toString(tag_id)+"' ");
                            if(visionTask.getDescriptor().equals(Integer.toString(tag_id))) {
                                Log.i(logTagTag,"will execute vision task "+visionTask);
                                visionTaskToExecute = true;
                            }
                        }
                    }
                    if(visionTaskToExecute) {  // if(isPartOfRobotVisualModel(tag_id))
                        Log.i(logTagTag,"checking on tag "+tag_id+": is part of robot visual model");
                    } else { // not part of something that we are looking for, so ignore
Log.i(logTagTag,"IGNORING TAG - not part of robot visual model - tag_id");
                        continue;
                    }
                    /* end Dev: part of robot visual model */

Log.i(logTagTag,"finished checking tag_id");
                    if( detector.hasMessage() )
                        System.out.println("Message   = "+detector.getMessage(i));
                    System.out.println("2D Image Location = "+locationPixel);

                    if( detector.is3D() ) {
Log.i(logTagTag,"start detector.getFiducialToCamera(i, targetToSensor);");
                        detector.getFiducialToCamera(i, targetToSensor);
Log.i(logTagTag,"after detector.getFiducialToCamera(i, targetToSensor);");

                        Vector3D_F64 transBoofCV_TtoS = targetToSensor.getTranslation();
                        Quaternion_F64 quatBoofCV_TtoS = new Quaternion_F64();
                        ConvertRotation3D_F64.matrixToQuaternion(targetToSensor.getR(), quatBoofCV_TtoS);
                        System.out.println("3D Location: targetToSensor : BoofCV frame : x = " + transBoofCV_TtoS.getX() + ", y = " + transBoofCV_TtoS.getY() + ", z = " + transBoofCV_TtoS.getZ());
                        System.out.println("3D Location: targetToSensor : BoofCV frame : qx = " + quatBoofCV_TtoS.x + ", qy = " + quatBoofCV_TtoS.y + ", qz = " + quatBoofCV_TtoS.z + ", qw = " + quatBoofCV_TtoS.w);

                        // 7210 is the translation and pose straight from BoofCV in tag-to-sensor


                        Se3_F64 sensorToTargetIn;
                        sensorToTargetIn = null;
                        Se3_F64 sensorToTarget;
                        sensorToTarget = targetToSensor.invert(sensorToTargetIn);
                        Vector3D_F64 transBoofCV_StoT = sensorToTarget.getTranslation();
                        Quaternion_F64 quatBoofCV_StoT = new Quaternion_F64();
                        ConvertRotation3D_F64.matrixToQuaternion(targetToSensor.getR(), quatBoofCV_TtoS);
//                        detectedFeaturesClient.reportDetectedFeature(8000+tag_id,
//                                transBoofCV_StoT.getX(), transBoofCV_StoT.getY(), transBoofCV_StoT.getZ(),
//                                quatBoofCV_StoT.x,quatBoofCV_StoT.y,quatBoofCV_StoT.z,quatBoofCV_StoT.w);


                        DenseMatrix64F transformation_fromBoofCVFiducialTagToSensor_toRobotSensorToTag
                                = new DenseMatrix64F(new double[][]{
                                {  0.0 ,  0.0 , -1.0 } ,
                                { -1.0 ,  0.0 ,  0.0 } ,
                                {  0.0 , +1.0 ,  0.0 } });


                        Se3_F64 targetToSensorViaTransform = new Se3_F64();
//                        DenseMatrix64F targetToSensorViaTransformRot = CommonOps.identity(3);
//                        CommonOps.mult(transformation_fromBoofCVFiducialTagToSensor_toRobotSensorToTag,targetToSensor.getR(),targetToSensorViaTransformRot);

                        Se3_F64 sensorToTargetViaTransform = new Se3_F64();
                        DenseMatrix64F sensorToTargetViaTransformRot = CommonOps.identity(3);
                        CommonOps.mult(transformation_fromBoofCVFiducialTagToSensor_toRobotSensorToTag,targetToSensor.getR(),sensorToTargetViaTransformRot);                        DenseMatrix64F sensorToTargetViaTransformRotInverted = CommonOps.identity(3);

//                        CommonOps.invert(sensorToTargetViaTransformRot);

                        sensorToTargetViaTransform.setRotation(sensorToTargetViaTransformRot);
//                        Vector3D_F64 sensorToTargetViaTransformTrans = new Vector3D_F64();
                        sensorToTargetViaTransform.setTranslation(transBoofCV_TtoS.getZ(), -1.0*transBoofCV_TtoS.getX(), -1.0*transBoofCV_TtoS.getY());
                        Quaternion_F64 sensorToTargetViaTransformQuat = new Quaternion_F64();
                        ConvertRotation3D_F64.matrixToQuaternion(sensorToTargetViaTransformRot, sensorToTargetViaTransformQuat);
//                        ConvertRotation3D_F64.setRotZ();
//                        detectedFeaturesClient.reportDetectedFeature(MARKER_OFFSET_INT+tag_id,
//                                sensorToTargetViaTransform.getX(), sensorToTargetViaTransform.getY(), sensorToTargetViaTransform.getZ(),
//                                sensorToTargetViaTransformQuat.x,sensorToTargetViaTransformQuat.y,sensorToTargetViaTransformQuat.z,sensorToTargetViaTransformQuat.w);

                        double[] eulerBefore=new double[]{0,0,0};
                        ConvertRotation3D_F64.matrixToEuler(sensorToTargetViaTransformRot,EulerType.YXY,eulerBefore);
                        double[] eulerAfter = new double[] {               eulerBefore[0], -1.0*eulerBefore[1], eulerBefore[2]};  // robot+Z+Y+Z = boof+Y-X+Y
                        ConvertRotation3D_F64.eulerToMatrix(EulerType.ZYZ, eulerAfter[0],   eulerAfter[1],      eulerAfter[2],    sensorToTargetViaTransformRot);
                        sensorToTargetViaTransform.setRotation(sensorToTargetViaTransformRot);
                        ConvertRotation3D_F64.matrixToQuaternion(sensorToTargetViaTransformRot, sensorToTargetViaTransformQuat);

//// TODO - timing here  c[camera_num]-f[frameprocessed]-i[iteration]-t[tagid]
Log.i(logTagTag,"after applying transformations");
                        /* Dev: part of robot visual model */
                        robots.put(singleDummyRobotId,robotFeatures);
                        if(isPartOfRobotVisualModel(tag_id)) {
                            DetectedTag detectedTag = new DetectedTag(tag_id,sensorToTargetViaTransform,sensorToTargetViaTransformQuat);
                            robotFeatures.add(detectedTag);
                        } else { // not part of something that we are looking for, so ignore
                            Log.i(logTagTag,"IGNORING TAG - not part of robot visual model - tag_id");
                            continue;
                        }
                        /* end Dev: part of robot visual model */

//// TODO - timing here  c[camera_num]-f[frameprocessed]-i[iteration]-t[tagid]
Log.i(logTagTag,"after detectedFeaturesClient.reportDetectedFeature");

/*

                            // transform the rotation to robot coordinate frame convention
                            DenseMatrix64F rotNeg90Y = new DenseMatrix64F(new double[][]{{0, 0, -1}, {0, 1, 0}, {1, 0, 0}});
                            DenseMatrix64F rotNeg90Z = new DenseMatrix64F(new double[][]{{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}});
                            DenseMatrix64F temp = CommonOps.identity(3);  // 3x3 zero-rotation matrix
                            DenseMatrix64F new_targetToSensor_r = CommonOps.identity(3); // initialise to 3x3 zero-rotation matrix - overwritten below
                            CommonOps.mult(rotNeg90Z, rotNeg90Y, temp);
                            CommonOps.mult(targetToSensor.getR(), temp, new_targetToSensor_r);
                            targetToSensor.setRotation(new_targetToSensor_r);

                        // transform the translation
                            Vector3D_F64 new_targetToSensor_t = new Vector3D_F64(0,0,0); // 3x1 zero-translation vector
                            GeometryMath_F64.mult(temp, targetToSensor.getTranslation(), new_targetToSensor_t);


                        Vector3D_F64 trans_TtoS = targetToSensor.getTranslation();
                        Quaternion_F64 quat_TtoS = new Quaternion_F64();
                        ConvertRotation3D_F64.matrixToQuaternion(targetToSensor.getR(), quat_TtoS);

//                        detectedFeaturesClient.reportDetectedFeature(1000+tag_id, trans_TtoS.getX(), trans_TtoS.getY(), trans_TtoS.getZ(), quat_TtoS.x,quat_TtoS.y,quat_TtoS.z,quat_TtoS.w);

//                        detectedFeaturesClient.reportDetectedFeature(2000+tag_id, trans_TtoS.getZ(), -trans_TtoS.getX(), -trans_TtoS.getY(), quat_TtoS.x,quat_TtoS.y,quat_TtoS.z,quat_TtoS.w);

//                        detectedFeaturesClient.reportDetectedFeature(22000+tag_id, trans_TtoS.getZ(), -trans_TtoS.getX(), -trans_TtoS.getY(), quat_TtoS.z,-quat_TtoS.x,-quat_TtoS.y,quat_TtoS.w);


                        sensorToTargetIn = null;
                        sensorToTarget = targetToSensor.invert(sensorToTargetIn);
                        Vector3D_F64 trans_StoT = sensorToTarget.getTranslation();
//                        VisualizeFiducial.drawCube(targetToSensor, param, detector.getWidth(i), 3, g2);
//                        VisualizeFiducial.drawLabelCenter(targetToSensor, param, "" + detector.getId(i), g2);
//                        org.ros.rosjava_geometry.Vector3 translation_to_tag_in_robot_convention
//                                = new org.ros.rosjava_geometry.Vector3(trans_StoT.getX(), trans_StoT.getY(), trans_StoT.getZ());
//                        org.ros.rosjava_geometry.Vector3 translation_to_tag_in_robot_convention = new org.ros.rosjava_geometry.Vector3(trans.getZ(), trans.getX(), trans.getY());
//                        org.ros.rosjava_geometry.Vector3 translation_to_tag_in_robot_convention = new org.ros.rosjava_geometry.Vector3(trans.getZ(), -trans.getX(), -trans.getY());
                        Quaternion_F64 quat_StoT = new Quaternion_F64();
                        ConvertRotation3D_F64.matrixToQuaternion(sensorToTarget.getR(), quat_StoT);
                        System.out.println("3D Location: sensorToTarget : BoofCV frame : x = "+trans_StoT.getX()+", y = "+trans_StoT.getY()+", z = "+trans_StoT.getZ());
                        System.out.println("3D Location: sensorToTarget : BoofCV frame : qx = "+quat_StoT.x+", qy = "+quat_StoT.y+", qz = "+quat_StoT.z+", qw = "+quat_StoT.w);

                        System.out.println("3D Location: targetToSensor = ");  System.out.println(targetToSensor);  System.out.println(quatBoofCV_TtoS);
                        System.out.println("3D Location: sensorToTarget = ");  System.out.println(sensorToTarget);  System.out.println(quat_StoT);



                        targetToSensor.setTranslation(new_targetToSensor_t);
//                        detectedFeaturesClient.reportDetectedFeature(3000+tag_id, trans_TtoS.getZ(), -trans_TtoS.getX(), -trans_TtoS.getY(), quat_TtoS.z,-quat_TtoS.x,-quat_TtoS.y,-quat_TtoS.w);

                        sensorToTargetIn = null;
                        sensorToTarget = targetToSensor.invert(sensorToTargetIn);
                        quat_StoT = new Quaternion_F64();
                        ConvertRotation3D_F64.matrixToQuaternion(sensorToTarget.getR(), quat_StoT);
                        trans_StoT = sensorToTarget.getTranslation();
//                        detectedFeaturesClient.reportDetectedFeature(4000+tag_id, trans_StoT.getX(), trans_StoT.getY(), trans_StoT.getZ(), quat_StoT.x,quat_StoT.y,quat_StoT.z,quat_StoT.w);
//                        detectedFeaturesClient.reportDetectedFeature(5000+tag_id, trans_StoT.getZ(), -trans_StoT.getX(), -trans_StoT.getY(), quat_StoT.x,quat_StoT.y,quat_StoT.z,quat_StoT.w);
//                        detectedFeaturesClient.reportDetectedFeature(6000+tag_id, trans_StoT.getZ(), -trans_StoT.getX(), -trans_StoT.getY(), quat_StoT.z, -quat_StoT.x,-quat_StoT.y,-quat_StoT.w);



//                        org.ros.rosjava_geometry.Quaternion quaternion_rotation_to_tag = new org.ros.rosjava_geometry.Quaternion(qz, -qx, -qy, qw);
//                        org.ros.rosjava_geometry.Quaternion quaternion_rotation_to_tag = new org.ros.rosjava_geometry.Quaternion(quat_StoT.x,quat_StoT.y,quat_StoT.z,quat_StoT.w);
//                        org.ros.rosjava_geometry.Quaternion quaternion_rotation_to_tag = new org.ros.rosjava_geometry.Quaternion(quat_StoT.z,-quat_StoT.x,-quat_StoT.y,quat_StoT.w);
//                        org.ros.rosjava_geometry.Quaternion quaternion_rotation_to_tag = new org.ros.rosjava_geometry.Quaternion(quat_StoT.z,quat_StoT.x,quat_StoT.y,quat_StoT.w);
                        DetectedFeature feature_hom = new DetectedFeature(APRIL_TAGS_KAESS_36_H_11,
                                Long.toString(tag_id),
//                                translation_to_tag_in_robot_convention,  // NOTE:  translation was good before; it's the rotation/orientation that was suspect
                                // Boof to ROS: Z to X, X to Y, Y to Z  --  Invert: mirror in Z-Y, so negate Z-Y
//                                new org.ros.rosjava_geometry.Vector3(transBoofCV_TtoS.getZ(), -transBoofCV_TtoS.getX(), -transBoofCV_TtoS.getY()),
//                                new org.ros.rosjava_geometry.Vector3(transBoofCV_TtoS.getX(), transBoofCV_TtoS.getY(), transBoofCV_TtoS.getZ()),
                                new org.ros.rosjava_geometry.Vector3(trans_StoT.getX(), trans_StoT.getY(), trans_StoT.getZ()),
                                new org.ros.rosjava_geometry.Quaternion(quat_StoT.x,quat_StoT.y,quat_StoT.z,quat_StoT.w));
                        detectedFeatures.add(feature_hom);
//                        detectedFeaturesClient.reportDetectedFeature(tag_id, trans_StoT.getX(), trans_StoT.getY(), trans_StoT.getZ(), quat_StoT.x,quat_StoT.y,quat_StoT.z,quat_StoT.w);
//                        detectedFeaturesClient.reportDetectedFeature(tag_id, trans_StoT.getZ(), -trans_StoT.getX(), -trans_StoT.getY(), quat_StoT.z,-quat_StoT.x,-quat_StoT.y,quat_StoT.w);
//                        detectedFeaturesClient.reportDetectedFeature(tag_id, transBoofCV_TtoS.getZ(), -transBoofCV_TtoS.getX(), -transBoofCV_TtoS.getY(), 0,0,0,1);
//                        detectedFeaturesClient.reportDetectedFeature(tag_id, transBoofCV_TtoS.getX(), transBoofCV_TtoS.getY(), transBoofCV_TtoS.getZ(), 0,0,0,1);

//                        detectedFeaturesClient.reportDetectedFeature(tag_id, trans_StoT.getX(), trans_StoT.getY(), trans_StoT.getZ(), quat_StoT.x,quat_StoT.y,quat_StoT.z,quat_StoT.w);

                        if (!poseKnown) {
//                            localiseFromAFeatureClient.localiseFromAFeature(tag_id, trans_StoT.getX(), trans_StoT.getY(), trans_StoT.getZ(), quat_StoT.x,quat_StoT.y,quat_StoT.z,quat_StoT.w);
//                            localiseFromAFeatureClient.localiseFromAFeature(tag_id, trans_StoT.getZ(), -trans_StoT.getX(), -trans_StoT.getY(), quat_StoT.z,-quat_StoT.x,-quat_StoT.y,quat_StoT.w);
//                            localiseFromAFeatureClient.localiseFromAFeature(tag_id, transBoofCV_TtoS.getZ(), -transBoofCV_TtoS.getX(), -transBoofCV_TtoS.getY(), 0,0,0,1);
                            localiseFromAFeatureClient.localiseFromAFeature(tag_id, trans_StoT.getX(), trans_StoT.getY(), trans_StoT.getZ(), quat_StoT.x,quat_StoT.y,quat_StoT.z,quat_StoT.w);
                        }
*/

                    } else {
//                        VisualizeFiducial.drawLabel(locationPixel, "" + detector.getId(i), g2);
                    }
                }
                Log.i(TAG, "onCameraFrame: after processing for "+detector.totalFound()+" tags found via BoofCV");

            } catch (Exception e) {
                Log.e(TAG, "onCameraFrame: exception running BoofCV fiducial: ", e);
                e.printStackTrace();
            }

        }
        Log.i(TAG, "onCameraFrame: after BoofCV segment ");

        // end BoofCV


        /* Dev: part of robot visual model */
        for(RobotId robotId_ : robots.keySet()) {
            double zRotationSum = 0.0;
            int numTagsForRobot = 0;
            double zRotationMean = 0.0;
            double xTranslationMean = 0.0;
            double yTranslationMean = 0.0;
            double zTranslationMean = 0.0;
            double zRotationPositive = 0.0;
            double zRotationPositivePrev = 0.0;
            double zRotationIncrementalMean = 0.0;
            for(DetectedTag detectedTag : robots.get(robotId_)) {
                numTagsForRobot++;
                xTranslationMean += detectedTag.getSensorToTargetViaTransform().getTranslation().getX();
                yTranslationMean += detectedTag.getSensorToTargetViaTransform().getTranslation().getY();
                zTranslationMean += detectedTag.getSensorToTargetViaTransform().getTranslation().getZ();
//                detectedTag.sensorToTargetViaTransform.getRotation();
                double[] eulerAngles = ConvertRotation3D_F64.quaternionToEuler(detectedTag.getSensorToTargetViaTransformQuat(), EulerType.ZYX, null);
                Log.i(robotId_.idString(), "Tag id = "+detectedTag.getTag_id()+" euler angles are "+eulerAngles[0]+" , "+eulerAngles[1]+" , "+eulerAngles[2]+"");
                double zRotation = eulerAngles[0];
                if(zRotation>Math.PI) {zRotation = zRotation - (Math.PI*2.0); } //e.g. convert 345 degrees to -15 degrees
                Log.i(robotId_.idString(), "Tag id = "+detectedTag.getTag_id()+" zRotation is "+zRotation);
                zRotationSum += zRotation;
                if(zRotation<0.0) {zRotationPositive=(Math.PI*2.0)+zRotation;} else {zRotationPositive=zRotation;}
                Log.i(robotId_.idString(), "Tag id = "+detectedTag.getTag_id()+" zRotationPositive is "+zRotationPositive);
                if(1 < numTagsForRobot) {
                    Log.w(robotId_.idString(), "UNWEIGHTED MEAN zRotationMean ONLY WORKS FOR 2 TAGS");
                    double diff = Math.abs(zRotationPositivePrev - zRotationPositive);
                    double diffDiv2 = diff/2.0;
                    Log.i(robotId_.idString(), "Tag id = "+detectedTag.getTag_id()+" diffDiv2 = "+diffDiv2);
                    if(diff > Math.PI) {
                        diffDiv2 = (Math.PI-diffDiv2);
                        Log.i(robotId_.idString(), "Tag id = "+detectedTag.getTag_id()+" diffDiv2 = "+diffDiv2);
                        if (zRotationPositivePrev > zRotationPositive) {
                            zRotationIncrementalMean = zRotationPositive - diffDiv2;
                        } else {
                            zRotationIncrementalMean = zRotationPositivePrev - diffDiv2;
                        }
                    } else {
                        if (zRotationPositivePrev > zRotationPositive) {
                            zRotationIncrementalMean = zRotationPositive + diffDiv2;
                        } else {
                            zRotationIncrementalMean = zRotationPositivePrev + diffDiv2;
                        }
                    }

                }
                zRotationPositivePrev=zRotationPositive;
            }
            if(numTagsForRobot>0) {
                xTranslationMean = xTranslationMean/numTagsForRobot;
                yTranslationMean = yTranslationMean/numTagsForRobot;
                zTranslationMean = zTranslationMean/numTagsForRobot;
                DenseMatrix64F meanSensorToTargetTransformRot;
//                zRotationMean = zRotationSum/numTagsForRobot;
//                Log.i(robotId_.idString(), "zRotationMean is "+zRotationMean);
//                meanSensorToTargetTransformRot = ConvertRotation3D_F64.eulerToMatrix(EulerType.ZYX, zRotationMean, 0.0, 0.0, null);
                Log.i(robotId_.idString(), "zRotationIncrementalMean is "+zRotationIncrementalMean);
                meanSensorToTargetTransformRot = ConvertRotation3D_F64.eulerToMatrix(EulerType.ZYX, zRotationIncrementalMean, 0.0, 0.0, null);
                Log.i(robotId_.idString(), "meanSensorToTargetTransformRot = "+meanSensorToTargetTransformRot);
                Se3_F64 meanSensorToTargetViaTransform = new Se3_F64();
                meanSensorToTargetViaTransform.setTranslation(xTranslationMean,yTranslationMean,zTranslationMean);
                meanSensorToTargetViaTransform.setRotation(meanSensorToTargetTransformRot);
                Quaternion_F64 meanSensorToTargetViaTransformQuat = new Quaternion_F64();
                ConvertRotation3D_F64.matrixToQuaternion(meanSensorToTargetTransformRot, meanSensorToTargetViaTransformQuat);
Log.i(robotId_.idString(), "estimated pose from "+numTagsForRobot+" tag detections");
            }
        }
        /* end Dev: part of robot visual model */


        Log.i(TAG, "onCameraFrame: starting OpenCV segment ");
        float  px_pixels = (float)(matGray.size().width/2.0);
        float  py_pixels = (float)(matGray.size().height/2.0);

    if( !running_native) {
        Log.i(TAG, "running_native == false: not running native code, not running OpenCV segment ");
    } else {
        Log.i(TAG, "starting OpenCV processing ");
//        String[] tags = aprilTagsUmichOneShot(matGray.getNativeObjAddr(), matRgb.getNativeObjAddr(), tagDetectorPointer, tagSize_metres, focal_length_in_pixels_x, focal_length_in_pixels_y, px_pixels, py_pixels);
        String[] tags = new String[0];
//        System.out.println("\\n\\n\\n ---------- hardcoding to ZERO AprilTags detections ---------- \\n\\n\\n");
//        String[] tags = new String[]{};

        System.out.println("---------- detected " + tags.length + " tags ----------------------------------------------------------------------------------------------------");

        Time timeNow = Date.nowAsTime();
        detectedFeatures.clear();
//        Matcher matcher;
        for (String tag : tags) {
            {
                System.out.println("-------------------------------------------------------");
                Log.i(TAG, "onCameraFrame: tag string = '" + tag + "'");
            }
//            matcher = tagPattern_trans_quat.matcher(tag);
//            if (!matcher.matches()) {
//                matcher = dataOutputPattern.matcher(tag);
//                if ( ! matcher.matches()) {
//                    System.out.print("---- ERROR: does not match either a detection or data to log ");
//                    continue;
//                }
//                logData(matcher.group(1),matcher.group(2));
//            }
            Matcher matcher = tagPattern_trans_quat.matcher(tag);
            {
                System.out.print("Pattern = '" + tagPattern_trans_quat.toString() + "'");
                System.out.print("--- matcher matches regex in the string? : ");
                System.out.println(matcher.matches());
            }
            String tagId = matcher.group(1);
            Integer tagId_integer = Integer.parseInt(tagId);
            int tagId_int = tagId_integer.intValue();
            System.out.print("---- matched tag_id=");
            System.out.print(tagId);
            System.out.print(", matched x=");
            System.out.print(matcher.group(2));
            System.out.println("---");

                System.out.print("MainActivityRosLess: onCameraFrame: detectedFeaturesClient is null: cannot report the poses of detected tags");
                System.out.println(tagId);
                System.out.println("-------------------------------------------------------");

        }
    } // end  if( running_native)
        Log.i(TAG, "after OpenCV segment ");

        if (screenLocked) {
            System.out.println("onCameraFrame: screenLocked = true at frame "+framesProcessed+": setting output matrices to black");
            Scalar blackScalar = new Scalar(0); //,CvType.CV_8UC4
            matRgb.setTo(blackScalar);
//// TODO - timing here  c[camera_num]-f[frameprocessed]-i[iteration]-t[tagid]
Log.i(logTag,"after matRgb.setTo(blackScalar);");
        }
        Log.i(TAG,"onCameraFrame: END: frame="+frameNumber);
        if (displayRgb) {
            return matRgb;
        } else {
            return matGray;
        }
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

    private Object taskQueueLock = new Object();

    private void removeExpiredVisionTasks() {
        ArrayList<VisionTask> toRemove = new ArrayList<VisionTask>();
        synchronized (this) {
            for (VisionTask task : taskQueue) {  // TODO - wrap this up in a VisionTaskQueue, and probably move to top or tail of the process , and look at e.g. ArrayBlockingQueue
                Log.i("MainActivityRosLess","removeExpiredVisionTask: vision task is now "+task);
                if(!task.canBeExecuted()) {
                    toRemove.add(task);     // could leave them in and only remove once a few have built up
                    Log.i("MainActivityRosLess","removeExpiredVisionTask: removed vision task "+task);
                }
                task.executed();
            }
            // TODO - not removing - for quicker test setup
            // TODO     taskQueue.removeAll(toRemove);
            // TODO - not removing - for quicker test setup
        }
    }

    /* Dev: part of robot visual model */
    private boolean isPartOfRobotVisualModel(int tag_id) {
        return tag_id == 170 || tag_id == 250 || tag_id == 290 || tag_id == 330;
    }

    private boolean isAnOutlier(DetectedFeature feature_) {
        return false;
    }

    private void calculateFocalLength_a(Camera camera) {
        double focal_length_in_pixels_x;
        double focal_length_in_pixels_y;
        if(null!=camera) {
            float focalLengthInMetres = camera.getParameters().getFocalLength();
            System.out.println("MainActivityRosLess: onCameraFrame: focalLengthInMetres="+focalLengthInMetres);
            float horizontalAngleView = camera.getParameters().getHorizontalViewAngle();
            System.out.println("MainActivityRosLess: onCameraFrame: horizontalAngleView="+horizontalAngleView);
            double focal_length_in_pixels = (camera.getParameters().getPictureSize().width * 0.5) / tan(horizontalAngleView * 0.5 * PI/180.0);
            System.out.println("MainActivityRosLess: onCameraFrame: focal_length_in_pixels="+focal_length_in_pixels);
        } else {
            System.out.println("MainActivityRosLess: onCameraFrame: AndroidCameraAdapterForDepricatedApi.getCameraInstance() returns null");
            boolean connected = false;
            for (int camIdx = 0; camIdx < Camera.getNumberOfCameras(); ++camIdx) {      //  see /mnt/nixbig/downloads/chulcher_ros_android_will_fork/android_core/openCVLibrary310/src/main/java/org/opencv/android/JavaCameraView.java:85
                Log.d(TAG, "Trying to open camera with new open(" + Integer.valueOf(camIdx) + ")");
                try {
                    camera = Camera.open(camIdx);
                    connected = true;
                } catch (RuntimeException e) {
                    Log.e(TAG, "Camera #" + camIdx + "failed to open: " + e.getLocalizedMessage());
                }
                if (connected) break;
            }
            if(connected) {
                float focalLengthInMetres = camera.getParameters().getFocalLength();
                System.out.println("MainActivityRosLess: onCameraFrame: focalLengthInMetres="+focalLengthInMetres);
                float horizontalAngleView = camera.getParameters().getHorizontalViewAngle();
                System.out.println("MainActivityRosLess: onCameraFrame: horizontalAngleView="+horizontalAngleView);
                focal_length_in_pixels_x = (camera.getParameters().getPictureSize().width * 0.5) / tan(horizontalAngleView * 0.5 * PI/180.0);
                System.out.println("MainActivityRosLess: onCameraFrame: focal_length_in_pixels_x="+focal_length_in_pixels_x);
                float verticalAngleView = camera.getParameters().getVerticalViewAngle();
                System.out.println("MainActivityRosLess: onCameraFrame: getVerticalViewAngle()="+verticalAngleView);
                focal_length_in_pixels_y = (camera.getParameters().getPictureSize().width * 0.5) / tan(verticalAngleView* 0.5 * PI/180.0);
                System.out.println("MainActivityRosLess: onCameraFrame: focal_length_in_pixels_y="+focal_length_in_pixels_y);
            } else {
                System.out.println("MainActivityRosLess: onCameraFrame: could not get a camera at all : using the Camera 1 API");
            }
        }
    }

    private void calculateFocalLength_b() {
        double focal_length_in_pixels_x;
        double focal_length_in_pixels_y;/*
        See  https://play.google.com/books/reader?id=hb8FCgAAQBAJ&printsec=frontcover&output=reader&hl=en_GB&pg=GBS.PA103.w.12.0.0  pp124-126
        */
                /* Get the focal length in pixels for AprilTags --> position calculation. */
        float[] estimatedFocusedDistances = {9000.0f,9000.0f,9000.0f};      // dummy values, overridden in cameraParameters().getFocusDistances(estimatedFocusedDistances)
        cameraParameters().getFocusDistances(estimatedFocusedDistances);    // focus distances in meters. the distances from the camera to where an object appears to be in focus. The object is sharpest at the optimal focus distance. The depth of field is the far focus distance minus near focus distance.param 'output' must be a float array with three elements. Near focus distance, optimal focus distance, and far focus distance will be filled in the array
        System.out.println("MainActivityRosLess: onCameraFrame: estimatedFocusedDistances = "+ Arrays.toString(estimatedFocusedDistances));
        MatOfDouble mProjectionCV = new MatOfDouble();
        mProjectionCV.create(3,3, CvType.CV_64FC1);
        final double fovAspectRatio = fieldOfViewX() / fieldOfViewY();
        double diagonalPixels = Math.sqrt( (Math.pow(matGray.size().width, 2.0)) + (Math.pow(matGray.size().width/fovAspectRatio, 2.0)) );
        double diagonalFov = Math.sqrt( (Math.pow(fieldOfViewX(), 2.0)) + (Math.pow(fieldOfViewY(), 2.0)) );
        double focalLengthPixels = diagonalPixels / (2.0 * Math.tan(0.5 * diagonalFov * Math.PI/180.0f ));
        focal_length_in_pixels_x = ( matGray.size().width / (2.0 * Math.tan(0.5 * fieldOfViewX() * Math.PI/180.0f)) );
        focal_length_in_pixels_y = ( matGray.size().height / (2.0 * Math.tan(0.5 * fieldOfViewY() * Math.PI/180.0f)) );
        System.out.println("MainActivityRosLess: onCameraFrame: "
                +"  OpenCV matrix: width pixels="+matGray.size().width+", height pixels="+matGray.size().height
                +", preview: width pixels="+cameraParameters().getPreviewSize().width+", height pixels="+cameraParameters().getPreviewSize().height
                +", zoom value="+cameraParameters().getZoom()+", zoom as percentage="+cameraParameters().getZoomRatios().get(cameraParameters().getZoom()).intValue()
                +", fieldOfView: X rads="+fieldOfViewX()+", Y rads="+fieldOfViewY()
                +", fovAspectRatio="+fovAspectRatio+", diagonalPixels="+diagonalPixels
                +", diagonalFov rads="+diagonalFov+", focalLengthPixels - diagonal="+focalLengthPixels);
        System.out.println("MainActivityRosLess: onCameraFrame: "
                +", focal length x = "+  focal_length_in_pixels_x
                +", focal length y = "+  focal_length_in_pixels_y );
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

    public void displayGrey() {
        displayRgb = false;
    }

    public void displayRgb() {
        displayRgb = true;
    }

    @Override
    public void setPose(double[] poseXyz, double[] orientationQuaternionXyzw_) {
        this.position = poseXyz;
        this.orientation = orientationQuaternionXyzw_;
        this.poseKnown = true;
    }

    public void setPose(Pose pose_) {
        setPose(
                new double[]{pose_.getPosition().getX(),pose_.getPosition().getY(),pose_.getPosition().getZ()},
                new double[]{pose_.getOrientation().getX(),pose_.getOrientation().getY(),pose_.getOrientation().getZ(),pose_.getOrientation().getW()});
    }

    public double[] getPositionXyz() {
        return position;
    }

    public double[] getOrientationQuaternionXyzw() {
        return orientation;
    }

    public boolean poseKnown(){
        return poseKnown;
    }


//    public native void salt(long matAddrGray, int nbrElem);
//    public native void canny(long matAddrGray);
//
//    public native long newTagDetectorKaess();   // Apriltags
//    public native void deleteTagDetectorKaess(long tagDetectorPointer);     // Apriltags
//    public native String[] aprilTags(long matAddrGray, long matAddrRgb, long tagDetectorPointer, double tagSize_metres, double fx_pixels, double fy_pixels);  // Apriltags
//
//    public native long newTagDetectorUmich();   // Apriltags
//    public native void deleteTagDetectorUmich(long tagDetectorPointer, long tagFamilyPointer);     // Apriltags
//    public native long aprilTagsUmich(       long matAddrGray, long matAddrRgb, long tagDetectorPointer, long tagFamilyPointer, double tagSize_metres, double fx_pixels, double fy_pixels);  // Apriltags_umich
//    public native String[] aprilTagsUmichOneShot(long matAddrGray, long matAddrRgb, long tagDetectorPointer,                        double tagSize_metres, float fx_pixels, float fy_pixels, float px_pixels, float py_pixels);  // Apriltags_umich
//

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

        checkPermissions();

        runImageProcessing = true;
        _cameraBridgeViewBase.enableView();
    }

    private void checkPermissions() {
            PermissionsChecker permissionsChecker = new PermissionsChecker(this);
            permissionsChecker.havePermissionAndRequest(Manifest.permission.ACCESS_WIFI_STATE, "This application requires access to wifi: please enable via Settings -> Apps (or similar) -> Your app -> Permissions");
            permissionsChecker.havePermissionAndRequest(Manifest.permission.CHANGE_WIFI_STATE, "This application requires access to wifi: please enable via Settings -> Apps (or similar) -> Your app -> Permissions");
            permissionsChecker.havePermissionAndRequest(Manifest.permission.INTERNET, "This application requires access to the internet: please enable via Settings -> Apps (or similar) -> Your app -> Permissions");
            permissionsChecker.havePermissionAndRequest(Manifest.permission.WAKE_LOCK, "This application requires access to wake the device: please enable via Settings -> Apps (or similar) -> Your app -> Permissions");
            permissionsChecker.havePermissionAndRequest(Manifest.permission.WRITE_EXTERNAL_STORAGE, "This application requires access to store data: please enable via Settings -> Apps (or similar) -> Your app -> Permissions");
            permissionsChecker.havePermissionAndRequest(Manifest.permission.ACCESS_FINE_LOCATION, "This application requires access the location: please enable via Settings -> Apps (or similar) -> Your app -> Permissions");
            permissionsChecker.havePermissionAndRequest(Manifest.permission.NFC, "This application requires access to Near Field Communications: please enable via Settings -> Apps (or similar) -> Your app -> Permissions");
            permissionsChecker.havePermissionAndRequest(Manifest.permission.CAMERA, "This application requires access to the camera: please enable via Settings -> Apps (or similar) -> Your app -> Permissions");
        }

    @Override
    public void stop() {
        runImageProcessing = false;
        _cameraBridgeViewBase.disableView();
    }


    public void allocateTo(String targetKey) {
        allocatedTargets.put(ROBOT_ALLOCATION_KEY,true);
    }

    @Override
    public void relocalise() {
        poseKnown   = false;
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

    public List<DetectedFeature> detectedFeatureList() {
        return detectedFeatures;
    }


    @Override
    public void imuData(Imu imu) {
System.out.println("imuData(Imu imu): relocalising");
        imu.getAngularVelocity();
        relocalise();
    }

//
//    boolean hasAccel = false;
//    boolean hasGyro = false;
//    boolean hasQuat = false;
//    private SensorManager sensorManager;
//    private ImuSensorListenerForCallback imuSensorListenerForCallback;
//    private ImuCallbackThread imuThread;
//    private int sensorDelay = 1;
//
//    private void detectImuSensors() {
//        List<Sensor> accelList = this.sensorManager.getSensorList(Sensor.TYPE_ACCELEROMETER);
//
//        if(accelList.size() > 0)
//        {
//            hasAccel = true;
//        }
//
//        List<Sensor> gyroList = this.sensorManager.getSensorList(Sensor.TYPE_GYROSCOPE);
//        if(gyroList.size() > 0)
//        {
//            hasGyro = true;
//        }
//
//        List<Sensor> quatList = this.sensorManager.getSensorList(Sensor.TYPE_ROTATION_VECTOR);
//        if(quatList.size() > 0)
//        {
//            hasQuat = true;
//        }
//    }
//
//    private void startImuSensorListenerAndThread() {
//        this.imuSensorListenerForCallback = new ImuSensorListenerForCallback(hasAccel, hasGyro, hasQuat, this);
//        this.imuThread = new ImuCallbackThread(this.sensorManager, imuSensorListenerForCallback, sensorDelay);
//        this.imuThread.start();
//    }


//    @Override
//    public void onPreviewFrame(byte[] frame, Camera arg1) {
//        Log.i(TAG, "----------------------------- onPreviewFrame -----------------------------");
//        GrayU8 image = ConvertBitmap.bitmapToGray(bitmap, (GrayU8)null, null);
//    }


    public byte[] last_frame_bytes(){
        return current_image_bytes;
    }

    protected VideoProcessing processing;

    public void last_frame_bytes(byte[] last_frame_bytes){
//        Log.i(TAG, "----------------------------- last_frame_bytes -----------------------------");
//        if( processing != null ) {
//            processing.convertPreview(last_frame_bytes,_cameraBridgeViewBase.camera());
//        }

        convertPreview(last_frame_bytes,_cameraBridgeViewBase.camera());  // updates the 'image' variable

// TODO - why was this in here? - start
//        FiducialDetector<GrayF32> detector = FactoryFiducial.squareBinary(
//                new ConfigFiducialBinary(0.1), ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 10), GrayF32.class);
////        detector.setLensDistortion(lensDistortion);
//        detector.detect(image);
//        Log.i(TAG, "last_frame_bytes: found "+detector.totalFound()+" tags via BoofCV");
// TODO - why was this in here? - end
    }
//
//    // Algorithm + id + pose + timestamp
//    class Detection implements VisualFeatureObservation {
//        String algorithm;
//        int id;
//        Quaternion orientation;
//        Vector3    position;
//        Time detectionDatetime;
//        // TODO - covariance or other uncertainty measure
//
//
//        public Detection(String algorithm_, int id_, Quaternion orientation_, Vector3 position_, Time detectionDatetime_) {
//            this.algorithm = algorithm_;
//            this.id = id_;
//            this.orientation = orientation_;
//            this.position = position_;
//            this.detectionDatetime = detectionDatetime_;
//        }
//
//        public Time detectionDatetime() {
//            return detectionDatetime;
//        }
//
//
//        @Override
//        public String getAlgorithm() {
//            return algorithm;
//        }
//
//        public String algorithm() {
//            return getAlgorithm();
//        }
//
//        @Override
//        public void setAlgorithm(String value) {
//            algorithm = value;
//        }
//
//        @Override
//        public int getId() {
//            return id;
//        }
//
//        public int id() {
//            return getId();
//        }
//
//        @Override
//        public void setId(int value) {
//            id = value;
//        }
//
//        @Override
//        public PoseStamped getPose() {
//            return null;
//        }
//        public PoseStamped pose() {
//            return getPose();
//        }
//
//        @Override
//        public void setPose(PoseStamped value) {
//        }
//
//        @Override
//        public RawMessage toRawMessage() {
//            return null;
//        }
//    }


    /**
     * Log some data: TODO - implement as a generic data publisher.
     */
    void logData(String dataLabel,String dataValue) {
        Log.i(TAG,"logData: logging data: label="+dataLabel+", value='"+dataValue+"'");
    }


/* start - copy from boofcv.android.gui.VideoRenderProcessing */ // TODO - use it properly
    // Type of BoofCV image
    ImageType<GrayF32> imageType = new ImageType<GrayF32>(GRAY,F32,1);       // TODO - hardcoded
    GrayF32 image = new GrayF32();  // TODO - hardcoded
    int previewRotation = 0;                                // TODO - hardcoded
    boolean flipHorizontal = false;                         // TODO - hardcoded

    byte[] current_image_bytes;

//    @Override
    public void convertPreview(byte[] bytes, Camera camera) {
        current_image_bytes = bytes;
//        if( thread == null )
//            return;
        if(null == matGray) {
            return;
        }
        int width_  = (int)Math.ceil(matGray.size().width);
        int height_ = (int)Math.ceil(matGray.size().height);
        image = image.createNew(width_,height_);

//        synchronized ( lockConvert ) {
            if( imageType.getFamily() == GRAY ) {
                ConvertNV21.nv21ToGray(bytes, image.width, image.height, (ImageGray) image,(Class) image.getClass());
//            } else if( imageType.getFamily() == ImageType.Family.PLANAR ) {
//                if (imageType.getDataType() == ImageDataType.U8)
//                    ConvertNV21.nv21ToMsRgb_U8(bytes, image.width, image.height, (Planar) image);
//                else if (imageType.getDataType() == ImageDataType.F32)
//                    ConvertNV21.nv21ToMsRgb_F32(bytes, image.width, image.height, (Planar) image);
//                else
//                    throw new RuntimeException("Oh Crap");
//            } else if( imageType.getFamily() == ImageType.Family.INTERLEAVED ) {
//                if( imageType.getDataType() == ImageDataType.U8)
//                    ConvertNV21.nv21ToInterleaved(bytes, image.width, image.height, (InterleavedU8) image);
//                else if( imageType.getDataType() == ImageDataType.F32)
//                    ConvertNV21.nv21ToInterleaved(bytes, image.width, image.height, (InterleavedF32) image);
//                else
//                    throw new RuntimeException("Oh Crap");
            } else {
                throw new RuntimeException("Unexpected image type: "+imageType);
            }

            if( previewRotation == 180 ) {
                if( flipHorizontal ) {
                    GImageMiscOps.flipVertical(image);
                } else {
                    GImageMiscOps.flipVertical(image);
                    GImageMiscOps.flipHorizontal(image);
                }
            } else if( flipHorizontal )
                GImageMiscOps.flipHorizontal(image);
//        }
//        // wake up the thread and tell it to do some processing
//        thread.interrupt();
    }

    private class MedianFilter {
        private boolean myResult;
        private String tag;
        private DetectedFeature feature;
        private DataTrack track;

        public MedianFilter(String tag, DetectedFeature feature, DataTrack track) {
            this.tag = tag;
            this.feature = feature;
            this.track = track;
        }

        boolean is() {
            return myResult;
        }

        public DetectedFeature getFeature() {
            return feature;
        }

        public MedianFilter invoke() {
            int medianFilterWindowSize = 5;                     // use 5 measurements in the median filter // TODO - hardcoding
            double neighbourhoodEuclideanSize = 0.1;            // allow 0.1m before reject  // TODO - hardcoding
            int[] matched = new int[medianFilterWindowSize];
            if(medianFilterWindowSize > track.sizeNow()) {
                myResult = true;
                return this;
            }
            DetectedFeature[] data = track.data(medianFilterWindowSize);
            DetectedFeature medianFeature = feature; // = data[medianFilterWindowSize-1];  // default to this most recent detection
            // TODO - effectively finding membership of distance-limited neighbourhood centred on each data point;  are other ways of determining a median
            // TODO - this could effectively smooth over data by using the most mediocre data values
            for(int i_ = 0; i_ < data.length; i_++) {
                DetectedFeature detectedFeature = data[i_];
                // have noise, so can't compare with equality; use e.g. 10cm , and 5.73 degrees (= 0.1 radians)
                if(null == detectedFeature) {
                    Log.e(TAG,"null == detectedFeature for "+i_+" on tag "+tag);}
                if(null == detectedFeature.translation_to_tag_in_robot_convention) {Log.e(TAG,"null == detectedFeature.translation_to_tag_in_robot_convention for "+i_+" on tag "+tag);}
                double xToCheck = detectedFeature.translation_to_tag_in_robot_convention.getX();
                double yToCheck = detectedFeature.translation_to_tag_in_robot_convention.getY();
                double zToCheck = detectedFeature.translation_to_tag_in_robot_convention.getZ();
                for(int j_=0; j_< data.length; j_++) {
                    if(j_ == i_) { continue; }
                    DetectedFeature detectedFeatureToCompare = data[j_];
                    if (euclideanBoundsCheck(neighbourhoodEuclideanSize, xToCheck, yToCheck, zToCheck, detectedFeatureToCompare)) { // close enough
                        matched[i_]++;
                    }
                }
            }
            int maxMatches  = 0;
            for(int i_=0; i_<matched.length; i_++) {
                if(matched[i_] >= maxMatches) {  // >= so that prefers more recent data - remember that matches are inexact
                    maxMatches = matched[i_];
                    medianFeature = data[i_];
                }
            }

            // TODO - could use the median value here as per median filtering
            //  OR could check whether the current data is close enough to the median and if it is, use it, otherwise use the median

            double xToCheck = medianFeature.translation_to_tag_in_robot_convention.getX();
            double yToCheck = medianFeature.translation_to_tag_in_robot_convention.getY();
            double zToCheck = medianFeature.translation_to_tag_in_robot_convention.getZ();
            double outlierLimit = neighbourhoodEuclideanSize * 2.0;
            if (!euclideanBoundsCheck(outlierLimit, xToCheck, yToCheck, zToCheck, feature)) { // close enough
                //continue; // filter out rubbish data
                feature = medianFeature;
            }
            myResult = false;
            return this;
        }

        private boolean euclideanBoundsCheck(double neighbourhoodEuclideanSize, double xToCheck, double yToCheck, double zToCheck, DetectedFeature detectedFeatureToCompare) {
            return (
            xToCheck+neighbourhoodEuclideanSize >= detectedFeatureToCompare.translation_to_tag_in_robot_convention.getX()
            ||
            xToCheck-neighbourhoodEuclideanSize <= detectedFeatureToCompare.translation_to_tag_in_robot_convention.getX()
            )
            &&
            (
            yToCheck+neighbourhoodEuclideanSize >= detectedFeatureToCompare.translation_to_tag_in_robot_convention.getY()
            ||
            yToCheck-neighbourhoodEuclideanSize <= detectedFeatureToCompare.translation_to_tag_in_robot_convention.getY()
            )
            &&
            (
            zToCheck+neighbourhoodEuclideanSize >= detectedFeatureToCompare.translation_to_tag_in_robot_convention.getZ()
            ||
            zToCheck-neighbourhoodEuclideanSize <= detectedFeatureToCompare.translation_to_tag_in_robot_convention.getZ()
            );
        }
    }

//    @Override
//    public void run() {
//        thread = Thread.currentThread();
//        while( !requestStop ) {
//            synchronized ( thread ) {
//                try {
//                    wait();
//                    if( requestStop )
//                        break;
//                } catch (InterruptedException e) {}
//            }
//
//            // swap gray buffers so that convertPreview is modifying the copy which is not in use
//            synchronized ( lockConvert ) {
//                T tmp = image;
//                image = image2;
//                image2 = tmp;
//            }
//
//            process(image2);
//
//            view.postInvalidate();
//        }
//        running = false;
//    }
/* end - copy from boofcv.android.gui.VideoRenderProcessing */

}

