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
import android.app.AlertDialog;
import android.content.ActivityNotFoundException;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.hardware.Camera;
import android.hardware.SensorManager;
import android.location.LocationManager;
import android.net.Uri;
import android.os.Bundle;
import android.os.Environment;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.view.SurfaceView;
import android.view.Window;
import android.widget.Toast;

import org.ddogleg.struct.FastQueue;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.opencv.android.Utils;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Scalar;
import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import android.view.Menu;
import android.view.MenuItem;
import android.view.WindowManager;
import android.view.MenuInflater;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.net.InetAddress;
import java.net.URI;
import java.net.UnknownHostException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.regex.Matcher;


import android.util.Log;

//import com.instacart.library.truetime.TrueTime;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.ros.rosjava_geometry.FrameTransform;
import org.ros.time.NtpTimeProvider;
import org.ros.time.TimeProvider;

import boofcv.abst.feature.associate.AssociateDescription;
import boofcv.abst.feature.associate.ScoreAssociation;
import boofcv.abst.feature.detdesc.DetectDescribePoint;
import boofcv.abst.feature.detect.interest.ConfigFastHessian;
import boofcv.abst.fiducial.FiducialDetector;
import boofcv.alg.color.ColorHsv;
import boofcv.alg.distort.LensDistortionNarrowFOV;
import boofcv.alg.distort.pinhole.LensDistortionPinhole;
import boofcv.alg.distort.radtan.LensDistortionRadialTangential;
import boofcv.alg.misc.GImageMiscOps;
import boofcv.android.gui.VideoProcessing;
import boofcv.core.encoding.ConvertNV21;
import boofcv.factory.feature.associate.FactoryAssociation;
import boofcv.factory.feature.detdesc.FactoryDetectDescribe;
import boofcv.factory.fiducial.ConfigFiducialBinary;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.factory.filter.binary.ConfigThreshold;
import boofcv.factory.filter.binary.ThresholdType;
import boofcv.struct.calib.CameraPinhole;
import boofcv.struct.calib.CameraPinholeRadial;
import boofcv.struct.feature.AssociatedIndex;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.ImageGray;
import boofcv.struct.image.ImageType;
import boofcv.struct.image.Planar;
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
import static william.chamberlain.androidvosopencvros.Algorithm.APRIL_TAGS_KAESS_36_H_11;
import static william.chamberlain.androidvosopencvros.DataExchange.tagPattern_trans_quat;
import static william.chamberlain.androidvosopencvros.DateAndTime.nowAsDate;

/**
 * @author chadrockey@gmail.com (Chad Rockey)
 * @author axelfurlan@gmail.com (Axel Furlan)
 */


public class MainActivity
        extends         RosActivity
        implements      CameraBridgeViewBase.CvCameraViewListener2,
        ActivityCompat.OnRequestPermissionsResultCallback,  // to deal with runtime permissions checks - from API 23 oward
        PosedEntity,                                    // Camera has a pose in the world; defaults to aligned with the map coordinate frame origin and axes.
        DetectedFeaturesHolder,
        ImuCallback,
        DimmableScreen, VariableResolution, VisionSource,
        ResilientNetworkActivity, VisionSource_WhereIs,
        FileLogger {

    //public static final double BOOFCV_TAG_SIZE_M = 0.189;  // 0.20  // 0.14 // 0.13;             ////  TODO - list of tags and sizes, and tag-groups and sizes
    public static final int FOUR_POINTS_REQUIRED_FOR_PNP = 4;
    public static final boolean LOCALISING_CAMERA_FROM_OBSERVED_FEATURES = false;
    public static final boolean TESTING_TRANSFORMATIONS_OF_TRANSFORMS = false;
    public static final String TIME_QUT_EDU_AU = "time.qut.edu.au";
    public static final String VOS_DATA_DIRECTORY_NAME = "VOS_data";
    public static final double[] NULL_POSITION = {0.0, 0.0, 1.0};
    public static final double[] NULL_ORIENTATION = {0.0, 0.0, 0.0, 1.0};
    private final LandmarkFeatureLoader landmarkFeatureLoader = new LandmarkFeatureLoader();

    HashMap<String,Boolean> allocatedTargets = new HashMap<String,Boolean>();

    public static final int CAMERA_FRONT_OR_BACK_INIT = CAMERA_ID_BACK;
    private static final boolean running_native = false;
    private static String determiningFreeFloorspace = null;


    private static final String TAG = "vos_aa1::MainActivity";
    public static final String MARKER_NAMESPACE = "apriltags_marker_publisher/tag_markers";

    // TODO - try native camera:  see https://stackoverflow.com/questions/16626343/what-is-the-difference-between-opencv-android-javacameraview-and-opencv-andro/28130605#28130605
    private CameraBridgeViewBase _cameraBridgeViewBase;


    private long tagDetectorPointer; // Apriltags

    private Mat matGray;
    private Mat matRgb;
//    private Mat mRgbaTransposed;      // rotate image as preview rotates with device
//    private Mat mRgbaFlipped;         // rotate image as preview rotates with device


    private double[] position    = NULL_POSITION;     // = new double[3]
    private double[] orientation = NULL_ORIENTATION; // = new double[4]
    private boolean  poseKnown   = false;

    long framesProcessed = 0;
    long frameNumber = 0;
//    PowerManager.WakeLock screenLock;


    VosTaskSet vosTaskSet;

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


    private ImuMonitoringPublisher imu_pub;
    private DetectedFeaturesClient detectedFeaturesClient;
    private SetPoseServer setPoseServer;
    private VisionSourceManagementListener visionSourceManagementListener;
    private LocaliseFromAFeatureClient localiseFromAFeatureClient;
    private WhereIsSubscriber          whereIsSubscriber;
    private RegisterVisionSourceClient registerVisionSourceClient;
    private LocaliseFromAFeatureServer localiseFromAFeatureServer;
    private VosTaskAssignmentSubscriberNode vosTaskAssignmentSubscriberNode;

    private SmartCameraTopLevelController smartCameraTopLevelController = new SmartCameraTopLevelController();
    private SmartCameraExtrinsicsCalibrator smartCameraExtrinsicsCalibrator = new SmartCameraExtrinsicsCalibrator();



//    private WhereIs whereIs;

    private LocationManager mLocationManager;
    private SensorManager mSensorManager;

    private boolean runImageProcessing = false;
    private boolean displayRgb = true;

    FeatureDataRecorderModeller featureDataRecorderModeller = new FeatureDataRecorderModeller();
    public static final int MIN_FRAME_SIZE_WIDTH_INIT = 100;
    public static final int MIN_FRAME_SIZE_HEIGHT_INIT = 100;
    public static final int MAX_FRAME_SIZE_WIDTH_INIT = 400;
    public static final int MAX_FRAME_SIZE_HEIGHT_INIT = 300;

    private SimpleDateFormat filenameDateFormat = new SimpleDateFormat("yyyy_MM_dd_HH_mm_ss");
    private SimpleDateFormat datafileLogDateFormat = filenameDateFormat;
    private static boolean RECORD_DATA = false;
    private static boolean RECORD_DATA_RECORDED = false;
    private static boolean RECORD_DATA_CONTINUOUS = false;
    private static int RECORD_FRAMES = 0;   // todo; get the feeling that there's a system aspect hiding here w.r.t. recording data related (1) to a set of observations (RECORD_DATA_X) (2) to a time period (RECORD_FRAMES)
    private static int CHECK_TIME_SYNCH = 10;
//    private FileOutputStream detectionDataOutputFile;
//    private OutputStreamWriter detectionDataOutputStreamWriter;
    private FileWriter detectionDataOutputFile = null;
    private File dataOutputDirectory = null;

    public MainActivity() {
        super("ROS Sensors Driver", "ROS Sensors Driver");
//        super("VOS AA1", "VOS AA1", URI.create("http://192.168.1.164:11311"));  //TODO // see http://wiki.ros.org/android/Tutorials/indigo/RosActivity , http://answers.ros.org/question/55874/skip-master-chooser/
    }

//    CameraManager cameraManager() {
//        return (CameraManager) this.getSystemService(CAMERA_SERVICE);
//    }


    private static final String ntpHost="172.19.63.161";           // TODO - pass the host portion of ROS_MASTER_URI from RosActivity

    @Override
    protected void onCreate(Bundle savedInstanceState)   // TODO - look at lifecycle doco and do this properly
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        checkPermissions();

//        try {
//            new Thread(new Runnable() {
//                @Override
//                public void run() {
//                    for (int i_ = 1; i_ <= 50; i_++) {
//                        try {
//                            TrueTime.build().withNtpHost(ntpHost).withRootDispersionMax(300).initialize();
//                            break;
//                        } catch (IOException e) {
//                            System.out.println("ERROR: IOException initialising TrueTime to NTP host " + ntpHost + ": " + e.getMessage());
//                            e.printStackTrace();
//                        }
//                    }
//                }
//            } ).start();
//        }
//        catch(RuntimeException e) { Log.e(TAG,"onCreate: exception initialising TrueTime to NTP host "+ntpHost+": "+e.getMessage(), e); }



        // normally called via smartCameraExtrinsicsCalibrator.estimateExtrinsicsFromObservations, which loads the data from the smartCameraExtrinsicsCalibrator.observations
        // private opencv_core.Mat calculateExtrinsics(double[] IMAGE_PIXEL_2D_DATA_POINTS_, double[] WORLD_3D_DATA_POINTS_, double imageWidth, double imageHeight) {
            System.out.println("----------\n----------\n----------\n----------\nsmartCameraExtrinsicsCalibrator.calculateExtrinsics start");
        //smartCameraExtrinsicsCalibrator.calculateExtrinsics(Hardcoding.pixel_coordinates__2017_12_29, Hardcoding.world_coordinates__2017_12_29, 352, 288);
            System.out.println("smartCameraExtrinsicsCalibrator.calculateExtrinsics end\n----------\n----------\n----------\n----------\n");


        mLocationManager = (LocationManager)this.getSystemService(Context.LOCATION_SERVICE);
        mSensorManager = (SensorManager)this.getSystemService(SENSOR_SERVICE);
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
//                PowerManager.PARTIAL_WAKE_LOCK, "MainActivity.onCameraFrame");

//        // Permissions for Android 6+
//        ActivityCompat.requestPermissions(MainActivity.this,
//                new String[]{Manifest.permission.CAMERA},
//                1);

        _cameraBridgeViewBase = (CameraBridgeViewBase) findViewById(R.id.main_surface);
        _cameraBridgeViewBase.setCameraIndex(CAMERA_FRONT_OR_BACK_INIT);  // front-facing camera is the user-side camera i.e. facing toward the front of the device from the user point of view and away from the front of the device from the device point of view, back-facing/rear-facing is usually the primary camera facing away from the user i.e. facing away from the rear of the device
        System.out.println("MainActivity: onCreate: after running _cameraBridgeViewBase.setCameraIndex("+CAMERA_FRONT_OR_BACK_INIT+")");
        System.out.println("MainActivity: onCreate: before running _cameraBridgeViewBase.setMaxFrameSize("+ MAX_FRAME_SIZE_WIDTH_INIT +","+ MAX_FRAME_SIZE_HEIGHT_INIT +")");
        _cameraBridgeViewBase.setMaxFrameSize(MAX_FRAME_SIZE_WIDTH_INIT, MAX_FRAME_SIZE_HEIGHT_INIT);  // http://stackoverflow.com/questions/17868954/android-opencv-how-to-set-camera-resolution-when-using-camerabridgeviewbase
        System.out.println("MainActivity: onCreate: after running _cameraBridgeViewBase.setMaxFrameSize("+ MAX_FRAME_SIZE_WIDTH_INIT +","+ MAX_FRAME_SIZE_HEIGHT_INIT +")");
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
//            tagDetectorPointer = newTagDetectorUmich();  // 2017_08_23 - remove to allow for Android API 16
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

//        System.out.println("MainActivity: onResume: before running AndroidCameraAdapterForDepricatedApi.setCameraToLowestResolution()");
//        AndroidCameraAdapterForDepricatedApi.setCameraToLowestResolution();
//        System.out.println("MainActivity: onResume: after running AndroidCameraAdapterForDepricatedApi.setCameraToLowestResolution()");

        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, _baseLoaderCallback);
//            _cameraBridgeViewBase.enableView();
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            _baseLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }



    public TimeProvider timeProvider = null;
    private NtpTimeProvider ntpTimeProvider=null;


    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) // configure nodes: config gets fed to an AsyncTask to start the Nodes in a Bound Service: see https://developer.android.com/reference/android/app/Service.html , https://developer.android.com/guide/components/processes-and-threads.html
    {
        Log.i("init", "start");
        final String NODE_NAMESPACE = Naming.cameraNamespace(getCamNum());
        Log.i("init", "NODE_NAMESPACE = "+NODE_NAMESPACE);

        vosTaskSet = new VosTaskSet();

        System.out.print("init: camNum=");System.out.print(getCamNum()); System.out.print("NODE_NAMESPACE = ");System.out.println(NODE_NAMESPACE);

        URI masterURI = getMasterUri();
        for (URI uri : possibleMasterUris()) {
        }
        /*
        TODO - looks like nodeMainExecutorService is being used to communicate the masterUri and camNum between the config screen/activity and MainActivity.
        The masterUri is then passed on to the configuration process for each node -
        probably in DefaultNodeMainExecutor.
        TODO - could extend NodeMainExecutorService to do the same masterUri checks as in ResilientNetworkActivity.
        TODO - can this get a connection to more than one ROS network by sending different masterUris to different node setups?
        not sure why I would (slow-but-reliable and fast-but-chittery maybe,
        for each node, and then bring the data together through some duplicate-exclusion filter
        before applying to the factor graph)
        , but maybe it could.
         RosActivity:
          public URI getMasterUri() {
            Preconditions.checkNotNull(nodeMainExecutorService);
            return nodeMainExecutorService.getMasterUri();
          }
         */
        System.out.print("init: masterURI = ");System.out.println(masterURI);
        Log.i("init","masterURI = "+masterURI);
        int currentapiVersion = android.os.Build.VERSION.SDK_INT;
        Log.i("init","android.os.Build.VERSION.SDK_INT = "+android.os.Build.VERSION.SDK_INT);

        int sensorDelay = 20000; // 20,000 us == 50 Hz for Android 3.1 and above
//        if(currentapiVersion <= android.os.Build.VERSION_CODES.HONEYCOMB){
//            sensorDelay = SensorManager.SENSOR_DELAY_UI; // 16.7Hz for older devices.  They only support enum values, not the microsecond version.
//        }

        Log.i("init","begin configuring ROS nodes");

        NodeConfiguration nodeConfigurationBase = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfigurationBase.setMasterUri(masterURI);
        InetAddress ntpServerAddress = null;
        try {
            List<String> ntpHostnames = new ArrayList<>();
                ntpHostnames.add(masterURI.getHost());
            ntpHostnames.add("131.181.33.25");
            ntpHostnames.add("192.168.86.100"); ntpHostnames.add("172.19.52.243"); ntpHostnames.add("172.19.20.202"); ntpHostnames.add("192.168.1.253"); ntpHostnames.add("192.168.1.164"); ntpHostnames.add("172.19.63.161"); ntpHostnames.add("time.qut.edu.au"); ntpHostnames.add("131.181.100.63"); //TIME_QUT_EDU_AU;
            timeProvider = null;
            for(String ntpHostname:ntpHostnames) {
                System.out.println("init: ntpHostname");
                InetAddress ntpServerAddress_ = InetAddress.getByName(ntpHostname);
                try {
                    Log.i(TAG, "init: initialiseNtpTimeProvider(" + ntpServerAddress_ + ") start");
                    NtpTimeProvider timeProvider_found = initialiseNtpTimeProvider(ntpServerAddress_);
                    if(null != timeProvider_found) {
                        timeProvider = timeProvider_found;
                        ntpServerAddress = ntpServerAddress_;
                        Log.i(TAG, "init: initialiseNtpTimeProvider(" + ntpServerAddress_ + ") succeeded.");
                        break;
                    } else {
                        Log.i(TAG, "init: initialiseNtpTimeProvider(" + ntpServerAddress_ + ") not found.");
                    }
                } catch (RuntimeException e) {
                    Log.w(TAG,"init: initialiseNtpTimeProvider("+ntpServerAddress_+") failed: will try the next NTP server in the list.");
                }
            }
            if(null == timeProvider) {
                Log.e(TAG,"init: could not initialise time from any of the NTP servers: throwing a RuntimeException.");
                throw new RuntimeException("init: could not initialise time from any of the NTP servers.");
            }
            nodeConfigurationBase.setTimeProvider(timeProvider);
            Log.i(TAG,"init: configured ROS TimeProvider NtpTimeProvider(\""+ntpServerAddress+"\", ), TimeProvider="+timeProvider);

            DateAndTime.configTimeProvider(timeProvider);
            Log.i(TAG,"init: configured Date:  Date.configTimeProvider( ROS TimeProvider NtpTimeProvider(\""+ntpServerAddress+"\", ), TimeProvider="+timeProvider+")" );
//            java.util.Date date1        = new java.util.Date();
//            org.ros.message.Time time1  = ntpTimeProvider.getCurrentTime();
//            java.util.Date trueTimeDate1 = TrueTime.now();
//            java.util.Date date2        = new java.util.Date();
//            org.ros.message.Time time2  = ntpTimeProvider.getCurrentTime();
//            java.util.Date trueTimeDate2 = TrueTime.now();
//            Log.i(TAG, "init: NtpTimeProvider: NtpTimeProvider time1 - java.util date1 = "+(time1.totalNsecs() - date1.getTime()*1000L*1000L)+"ns = "+(time1.totalNsecs()/(1000L*1000L) - date1.getTime())+"ms");
//            Log.i(TAG, "init: NtpTimeProvider: NtpTimeProvider time2 - java.util date2 = "+(time2.totalNsecs() - date2.getTime()*1000L*1000L)+"ns = "+(time2.totalNsecs()/(1000L*1000L) - date2.getTime())+"ms");
//            Log.i(TAG, "init: NtpTimeProvider: NtpTimeProvider time1 - TrueTime.now() date1 = "+(time1.totalNsecs() - trueTimeDate1.getTime()*1000L*1000L)+"ns = "+(time1.totalNsecs()/(1000L*1000L) - trueTimeDate1.getTime())+"ms");
//            Log.i(TAG, "init: NtpTimeProvider: NtpTimeProvider time2 - TrueTime.now() date2 = "+(time2.totalNsecs() - trueTimeDate2.getTime()*1000L*1000L)+"ns = "+(time2.totalNsecs()/(1000L*1000L) - trueTimeDate2.getTime())+"ms");
        } catch(UnknownHostException e){Log.e(TAG,"init: failed to configure ROS TimeProvider due to unknown host in  InetAddress.getByName( "+masterURI+").getHost() ))  : "+e, e); }



        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
//            NodeConfiguration nodeConfiguration8 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
//            nodeConfiguration8.setMasterUri(masterURI);
            NodeConfiguration nodeConfiguration8 = NodeConfiguration.copyOf(nodeConfigurationBase);
            nodeConfiguration8.setNodeName(NODE_NAMESPACE+"detectedfeatures_serviceclient_node");
            this.detectedFeaturesClient = new DetectedFeaturesClient();
            detectedFeaturesClient.setCameraFrameId(Naming.cameraFrameId(getCamNum()));
            detectedFeaturesClient.setPosedEntity(this);
            nodeMainExecutor.execute(this.detectedFeaturesClient, nodeConfiguration8);
        }


        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
//            NodeConfiguration nodeConfiguration8 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
//            nodeConfiguration8.setMasterUri(masterURI);
            NodeConfiguration nodeConfiguration8 = NodeConfiguration.copyOf(nodeConfigurationBase);
            nodeConfiguration8.setNodeName(NODE_NAMESPACE+"pose_server");
            this.setPoseServer = new SetPoseServer();
            setPoseServer.setNodeNamespace(NODE_NAMESPACE);
            setPoseServer.setPosedEntity(this);
            nodeMainExecutor.execute(this.setPoseServer, nodeConfiguration8);
        }

//        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
//            NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
//            nodeConfiguration.setMasterUri(masterURI);
//            nodeConfiguration.setNodeName(NODE_NAMESPACE+"vos_task_requests");
//            this.whereIsSubscriber = new WhereIsSubscriber(this);
//            whereIsSubscriber.setNodeNamespace(NODE_NAMESPACE);
//            whereIsSubscriber.setPosedEntity(this);
//            nodeMainExecutor.execute(this.whereIsSubscriber, nodeConfiguration);
//        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
//            NodeConfiguration nodeConfiguration8 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
//            nodeConfiguration8.setMasterUri(masterURI);
            NodeConfiguration nodeConfiguration8 = NodeConfiguration.copyOf(nodeConfigurationBase);
            nodeConfiguration8.setNodeName(NODE_NAMESPACE+"vision_source_management_topic_listener");
            this.visionSourceManagementListener = new VisionSourceManagementListener();
            visionSourceManagementListener.setNodeNamespace(NODE_NAMESPACE);
            visionSourceManagementListener.setDimmableScreen(this);
            visionSourceManagementListener.setVariableResolution(this);
            visionSourceManagementListener.setVisionSource(this);
            nodeMainExecutor.execute(this.visionSourceManagementListener, nodeConfiguration8);
        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
//            NodeConfiguration nodeConfiguration8 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
//            nodeConfiguration8.setMasterUri(masterURI);
            NodeConfiguration nodeConfiguration8 = NodeConfiguration.copyOf(nodeConfigurationBase);
            nodeConfiguration8.setNodeName(NODE_NAMESPACE+"localiseFromAFeature_serviceclient_node");
            this.localiseFromAFeatureClient = new LocaliseFromAFeatureClient();
            localiseFromAFeatureClient.setCameraFrameId(Naming.cameraFrameId(getCamNum()));
            localiseFromAFeatureClient.setPosedEntity(this);
            nodeMainExecutor.execute(this.localiseFromAFeatureClient, nodeConfiguration8);
        }



        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
//            NodeConfiguration nodeConfiguration8 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
//            nodeConfiguration8.setMasterUri(masterURI);
            NodeConfiguration nodeConfiguration8 = NodeConfiguration.copyOf(nodeConfigurationBase);
            nodeConfiguration8.setNodeName(NODE_NAMESPACE+"registervisionsource_serviceclient_node");
            this.registerVisionSourceClient = new RegisterVisionSourceClient();
            registerVisionSourceClient.setBaseUrl(Naming.cameraNamespace(getCamNum()));
            // TODO - do not set up the subscriber - having this setup in a close loop with creating the service does not work cleanly
            // TODO - could set the Subscribers up on demand - just frame as network config
            // TODO - this.whereIsSubscriber = new WhereIsSubscriber(this);
            // TODO - registerVisionSourceClient.setWhereIsSubscriber(whereIsSubscriber);
            nodeMainExecutor.execute(this.registerVisionSourceClient, nodeConfiguration8);
        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
//            NodeConfiguration nodeConfiguration8 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
//            nodeConfiguration8.setMasterUri(masterURI);
            NodeConfiguration nodeConfiguration8 = NodeConfiguration.copyOf(nodeConfigurationBase);
            nodeConfiguration8.setNodeName(NODE_NAMESPACE+"localiseFromAFeature_serviceserver_node");
            this.localiseFromAFeatureServer = new LocaliseFromAFeatureServer();
            localiseFromAFeatureServer.setNodeNamespace(Naming.cameraNamespace(getCamNum()));
            localiseFromAFeatureServer.setPosedEntity(this);
            localiseFromAFeatureServer.setDetectedFeaturesHolder(this);
            nodeMainExecutor.execute(this.localiseFromAFeatureServer, nodeConfiguration8);
        }
//        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
//            NodeConfiguration nodeConfiguration8 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
//            nodeConfiguration8.setMasterUri(masterURI);
//            nodeConfiguration8.setNodeName(NODE_NAMESPACE+"where_is_alg_desc");
//            this.whereIs = new WhereIs();
//            whereIs.setNodeNamespace(Naming.cameraNamespace(getCamNum()));
//            nodeMainExecutor.execute(this.whereIs, nodeConfiguration8);
////  TODO  -  IMAGEPUBLISHER
//        }

        /*  subscriber to VOS Server -
            1) the VOS Server publishes vision tasks on /cam_607/vos_task_assignment_subscriber as WhereIsAsPub messages
            2) the VosTaskAssignmentSubscriberNode.visionSource_WhereIs is subscribed to  e.g.  cam_607/vos_task_assignment_subscriber
            3) the VosTaskAssignmentSubscriberNode.visionSource_WhereIs takes care of putting the vision tasks into vosTaskSet, by ...
            3.1) ... calling  MainActivity.dealWithRequestForInformation(WhereIsAsPub message)
        */
        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
//            NodeConfiguration nodeConfiguration8 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
//            nodeConfiguration8.setMasterUri(masterURI);
            NodeConfiguration nodeConfiguration8 = NodeConfiguration.copyOf(nodeConfigurationBase);
            nodeConfiguration8.setNodeName(NODE_NAMESPACE+"_vos_vision_tasks");
            this.vosTaskAssignmentSubscriberNode = new VosTaskAssignmentSubscriberNode();
            this.vosTaskAssignmentSubscriberNode.setNodeNamespace(Naming.cameraNamespace(getCamNum()));
            this.vosTaskAssignmentSubscriberNode.setVisionSource_WhereIs(this);
            this.vosTaskAssignmentSubscriberNode.setSmartCameraTopLevelController(this.smartCameraTopLevelController);
            setupSmartCameraExtrinsicsCalibrator();

            nodeMainExecutor.execute(this.vosTaskAssignmentSubscriberNode, nodeConfiguration8);
        }
        Log.i("init","finish configuring ROS nodes");

        Log.i("init","start configuring fixed camera poses");
        Hardcoding.fixedCameraPose(getCamNum(),this);
        Log.i("init","finish configuring fixed camera poses");

        Log.i("init", "finished");
    }

    private NtpTimeProvider initialiseNtpTimeProvider(InetAddress ntpServerAddress) {
        ntpTimeProvider = new NtpTimeProvider(ntpServerAddress, nodeMainExecutorService.getScheduledExecutorService());
        timeProvider = ntpTimeProvider;
        try {
            System.out.println("initialiseNtpTimeProvider: start: address="+ntpServerAddress.toString());
            ntpTimeProvider.updateTime();
            Log.i(TAG,"initialiseNtpTimeProvider: NtpTimeProvider: ntpTimeProvider.updateTime() succeeded.");
            return ntpTimeProvider;
        } catch (java.net.SocketTimeoutException e) {
            Log.e(TAG,"initialiseNtpTimeProvider: NtpTimeProvider: ntpTimeProvider.updateTime(): "+ntpServerAddress.toString()+": timeout exception"+e, e);
            return null;
        } catch (IOException e) {
            Log.e(TAG,"initialiseNtpTimeProvider: NtpTimeProvider: ntpTimeProvider.updateTime(): "+ntpServerAddress.toString()+": exception"+e, e);
            throw new RuntimeException("initialiseNtpTimeProvider: initialising ntpTimeProvider failed with address "+ntpServerAddress.toString(),e);
        }
    }

    void setupSmartCameraExtrinsicsCalibrator() {
        this.vosTaskAssignmentSubscriberNode.addRobotStatusChangeListener(this.smartCameraExtrinsicsCalibrator);
        this.smartCameraExtrinsicsCalibrator.setRobotPoseMeasure(this.vosTaskAssignmentSubscriberNode);
        this.smartCameraExtrinsicsCalibrator.setRobotGoalPublisher(this.vosTaskAssignmentSubscriberNode);
        this.smartCameraExtrinsicsCalibrator.setPoseEstimator2D3D(this.vosTaskAssignmentSubscriberNode);
        this.smartCameraExtrinsicsCalibrator.setFileLogger(this);
        this.smartCameraExtrinsicsCalibrator.setPosedEntity(this);
    }


    public URI[] possibleMasterUris() {
        URI[] rosNetworkingOptions  // start with the Master URIs, then work outward to other network options
            = new URI[]{
                URI.create("http://192.168.1.144:11311/"),
                URI.create("http://192.168.1.164:11311/"),
                URI.create("http://192.168.1.252:11311/"),
                URI.create("http://localhost:11311/")
        };
        return rosNetworkingOptions;
    }


    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.activity_main, menu);
        return super.onCreateOptionsMenu(menu);
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        if(item.getItemId() == R.id.menu_help) {
            AlertDialog.Builder builder = new AlertDialog.Builder(this);
            builder.setCancelable(true);
            builder.setTitle(getResources().getString(R.string.help_title));
            builder.setMessage(getResources().getString(R.string.help_message));
            builder.setInverseBackgroundForced(true);
            builder.setNegativeButton(getResources().getString(R.string.help_ok),
                    new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            dialog.dismiss();
                        }
                    });
            builder.setNeutralButton(getResources().getString(R.string.help_wiki),
                    new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            Intent i = new Intent(Intent.ACTION_VIEW);
                            Uri u = Uri.parse("http://www.ros.org/wiki/android_sensors_driver");
                            try {
                                // Start the activity
                                i.setData(u);
                                startActivity(i);
                            } catch (ActivityNotFoundException e) {
                                // Raise on activity not found
                                Toast toast = Toast.makeText(MainActivity.this, "Browser not found.", Toast.LENGTH_SHORT);
                                toast.show();
                            }
                        }
                    });
            builder.setPositiveButton(getResources().getString(R.string.help_report),
                    new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            Intent i = new Intent(Intent.ACTION_VIEW);
                            Uri u = Uri.parse("https://github.com/ros-android/android_sensors_driver/issues/new");
                            try {
                                // Start the activity
                                i.setData(u);
                                startActivity(i);
                            } catch (ActivityNotFoundException e) {
                                // Raise on activity not found
                                Toast toast = Toast.makeText(MainActivity.this, "Browser not found.", Toast.LENGTH_SHORT);
                                toast.show();
                            }
                        }
                    });
            AlertDialog alert = builder.create();
            alert.show();

        }
        return super.onOptionsItemSelected(item);
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
//        try { detectionDataOutputStreamWriter.close(); detectionDataOutputFile.close(); }
        closeDataOutputFile();
    }

    private void closeDataOutputFile() {
        if(null != detectionDataOutputFile) {
            try { detectionDataOutputFile.close();
            } catch (IOException e) { Log.e("onDestroy", "MainActivity: onDestroy: could not close the detectionDataOutputFile data file stream or file.", e);
            }
        }
    }

    public void disableCamera() {
        if (_cameraBridgeViewBase != null) {
            _cameraBridgeViewBase.disableView();
        }
    }

    public void onCameraViewStarted(int width, int height) {
        System.out.println("MainActivity: onCameraViewStarted("+width+","+height+"): start");
//        mRgbaFlipped = new Mat(height,width, CvType.CV_8UC4);
//        mRgbaTransposed = new Mat(height,width, CvType.CV_8UC4);
        matRgb = new Mat(height,width, CvType.CV_8UC4);
        System.out.println("MainActivity: onCameraViewStarted("+width+","+height+"): end");
    }

    public void onCameraViewStopped() {
        if( null != matGray) { matGray.release(); }
        if( null != matRgb) { matRgb.release(); }
//        if( null != mRgbaFlipped) { mRgbaFlipped.release(); }
//        if( null != mRgbaTransposed) { mRgbaTransposed.release(); }
    }


    /*** implement VisionSource_WhereIs ***********************************************************/

    public void dealWithRequestForInformation(WhereIsAsPub message){
        Log.i(TAG,"dealWithRequestForInformation(WhereIsAsPub message) : "+message.getRobotId()+", "+message.getAlgorithm()+", "+message.getDescriptor()+", "+message.getRequestId()+", "+message.toString() );
        synchronized (this) {
            vosTaskSet.addVisionTaskToQueue(message);
        }
    }

    /*** end implement VisionSource_WhereIs ***********************************************************/



    boolean screenLocked = false;
    boolean registeredAsVisionSource = false;
////    List<DetectedFeature> detectedFeatures = Collections.synchronizedList(new ArrayList<DetectedFeature>());
    // avoid synchronising on the list with e.g.   synchronized (detectedFeatures) { for(detectedFeature: detectedFeatures) {...} }
    // TODO - see http://www.codejava.net/java-core/collections/understanding-collections-and-thread-safety-in-java , https://docs.oracle.com/javase/7/docs/api/java/util/concurrent/CopyOnWriteArrayList.html
    List<DetectedFeature> detectedFeatures = new java.util.concurrent.CopyOnWriteArrayList<DetectedFeature>();


    List<List<DetectedTag>> robotFeatureTracking = new ArrayList<List<DetectedTag>>();  // TODO - make this a queue or something that won't overrun
    List<Point2D_F64> robotFeatureTrackingAverages = new ArrayList<Point2D_F64>();      // TODO - make this a queue or something that won't overrun
    Object robotFeatureTrackingMonitor = new Object();

    /*** implement CameraBridgeViewBase.CvCameraViewListener2 *************************************/
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {

        incrementFrameCounter();

        java.util.Date imageFrameTime = currentTimeSafe(CHECK_TIME_SYNCH > 0);
        if( CHECK_TIME_SYNCH > 0 ){ CHECK_TIME_SYNCH--; }

        //        verifyTime();
        Log.i(TAG,"onCameraFrame: START: cameraNumber="+getCamNum()+": frame="+frameNumber);

        if(!readyToProcessImages) {
            Log.i(TAG,"onCameraFrame: readyToProcessImages is false: returning image without processing.");
            return inputFrame.gray();
        }

        incrementFrameProcessedCounter();

        String logTag = "c"+getCamNum()+"-f"+framesProcessed;

        Log.i(logTag,"start frame");   //// TODO - timing here  c[camera_num]-f[frameprocessed]

        if(RECORD_DATA) {
            RECORD_DATA_RECORDED = false;
            Log.i(TAG,"onCameraFrame: RECORD_DATA=true: going to be doing some recording on this frame: "+logMessage_recordDataStatus());
        }

        registerAsVisionSource(logTag); //// TODO - move into control loop

        focalLengthCorrections();

        rotationCorrection();

        matGray = inputFrame.gray();
        matRgb  = inputFrame.rgba();
        System.out.println("MainActivity: onCameraFrame("+matGray.size().width+","+matGray.size().height+"): start");

        exitIfNotImageProcessing();

        Camera camera = AndroidCameraAdapterForDepricatedApi.getCameraInstance();
//        if(null == camera) {
//            try { camera = _cameraBridgeViewBase.camera(); }
//            catch (Exception e) {
//                System.out.println("MainActivity: onCameraFrame: exception in camera = _cameraBridgeViewBase.camera() : "+e.getMessage());
//            }
//        }

        FocalLengthCalculator focalLengthCalc = new FocalLengthCalculator();

        rotateImage();

        // TODO - try reducing image size to increase framerate , AND check /Users/will/Downloads/simbaforrest/cv2cg_mini_version_for_apriltag , https://github.com/ikkiChung/MyRealTimeImageProcessing , http://include-memory.blogspot.com.au/2015/02/speeding-up-opencv-javacameraview.html , https://developer.qualcomm.com/software/fastcv-sdk , http://nezarobot.blogspot.com.au/2016/03/android-surfacetexture-camera2-opencv.html , https://www.youtube.com/watch?v=nv4MEliij14 ,

        // start BoofCV
        byte[] current_image_bytes = last_frame_bytes();
        boolean current_image_bytes_is_not_null = (null != current_image_bytes);
            if(!current_image_bytes_is_not_null) {Log.i(logTag, "current_image_bytes is null"); }

        vosTaskSet.removeExpiredVisionTasks();

        CalcImageDimensions calcImgDim = new CalcImageDimensions();

        boolean thereIsAVisionTaskToExecute     = vosTaskSet.isThereAVisionTaskToExecute();
        if(!thereIsAVisionTaskToExecute) { Log.i(logTag, "no vision task to execute"); }


        convertPreviewForBoofCV(last_frame_bytes(), camera);                                 // NOTE: inside-out: this is a BoofCV conversion because we know that we're using a BoofCV algorithm
        Log.i(logTag,"onCameraFrame: IMAGE SIZE CHECK: byte[] size=last_frame_bytes().length="+last_frame_bytes().length+" : image.height="+image.height+", image.width="+image.width+", area="+(image.height*image.width)+" : matGray.height()="+matGray.height()+", matGray.width()="+matGray.width()+", area="+(matGray.height()*matGray.width()));
        Log.i(logTag,"onCameraFrame: IMAGE RECORD: RECORD_DATA="+RECORD_DATA);
        Log.i(logTag,"onCameraFrame: IMAGE RECORD: RECORD_FRAMES="+RECORD_FRAMES);
//        boolean JFDI = true;
        if(RECORD_FRAMES > 0) { // || JFDI) {  // save 'image' to file
            Log.i(logTag,"onCameraFrame: IMAGE : RECORD_FRAMES="+RECORD_FRAMES);
            RECORD_FRAMES--;
            Log.i(logTag,"onCameraFrame: IMAGE : RECORD_FRAMES="+RECORD_FRAMES);
            String prefix_="BoofCV";
            File dest = fileInExternalStorage(
                    imageFrameTime,
                    filenameDateFormat.format(imageFrameTime) + "_" + prefix_ + "_image.png");
            FileOutputStream out = null;
            Log.i(logTag,"onCameraFrame: IMAGE : RECORD_DATA: destination file = "+dest.getAbsolutePath());
            Log.i("saveImageDataToFile", "MainActivity: saveImageDataToFile: destination file = "+dest.getAbsolutePath());
            try {
                Log.i("saveImageDataToFile", "MainActivity: saveImageDataToFile: open FileOutputStream: start.");
                out = new FileOutputStream(dest);
                Log.i("saveImageDataToFile", "MainActivity: saveImageDataToFile: open FileOutputStream: end.");
                Log.i("saveImageDataToFile", "MainActivity: saveImageDataToFile: bitmap compress to PNG and output: start.");
                Bitmap bmp = null;
                int min=128, max=128;
                try {
                    bmp = Bitmap.createBitmap(image.width, image.height, Bitmap.Config.ARGB_8888); //Bitmap createBitmap(int width, int height, Config config)

                    for( int y = 0; y < image.height; y++ ) {
                        int indexIn = y*image.width;
                        int indexOut = image.startIndex + y*image.stride;

                        for( int x = 0; x < image.width; x++ ) {
                            float val_ = image.data[ indexIn++ ];
                            int a = (int)val_;
                            if(a<min){min=a;Log.i("IMAGE","min="+min);} if(a>max){max=a;Log.i("IMAGE","max="+max);}
                            int colour = Color.rgb(a,a,a);
                            bmp.setPixel(x,y,colour);           //setPixel(int x, int y, @ColorInt int color)
                        }
                    }
                } catch (CvException e) {
                    Log.e("saveImageDataToFile", "MainActivity: saveImageDataToFile: convert to bitmap: exception: "+e.getMessage(), e);
                }
                bmp.compress(Bitmap.CompressFormat.PNG, 100, out); // bmp is your Bitmap instance
                // PNG is a lossless format, the compression factor (100) is ignored
                Log.i("saveImageDataToFile", "MainActivity: saveImageDataToFile: bitmap compress to PNG and output: end.");
            } catch (Exception e) {
                e.printStackTrace();
                Log.e("saveImageDataToFile", "MainActivity: saveImageDataToFile: could not output - exception: "+e.getMessage(), e);
            } finally {
                try {
                    if (out != null) {
                        out.close();
                        Log.i("saveImageDataToFile", "MainActivity: saveImageDataToFile: out FileOutputStream closed.");
                    }
                } catch (IOException e) {
                    Log.e("saveImageDataToFile", "MainActivity: saveImageDataToFile: could not close out FileOutputStream: "+e.getMessage(), e);
                    e.printStackTrace();
                }
            }
//            RECORD_DATA_RECORDED = true;
        }


        if(current_image_bytes_is_not_null  &&  thereIsAVisionTaskToExecute) {
            /* partial implementation: part of robot visual model */
            HashMap<RobotId, List<DetectedTag>> robotsDetected = new HashMap<RobotId,List<DetectedTag>>();
            RobotId singleDummyRobotId = new RobotId(999999); // new RobotId("dummy robot id");
            List<DetectedTag> robotFeatures = new ArrayList<DetectedTag>();
            List<DetectedTag> landmarkFeatures = new ArrayList<DetectedTag>();
            /* end partial implementation: part of robot visual model */

            convertPreviewForBoofCV(last_frame_bytes(), camera);                                 // NOTE: inside-out: this is a BoofCV conversion because we know that we're using a BoofCV algorithm

            VosTaskSet thingsIShouldBeLookingFor = vosTaskSet;


            if(thingsIShouldBeLookingFor.includes(Algorithm.GEOMETRIC)) {                           // TODO - backwards - should spec the vision tasks, then check the pipelines for each , rather than enumerate all the algorithm types
                Log.i(TAG, "onCameraFrame: start GEOMETRIC feature processing.");
            } else {
                Log.i(TAG, "onCameraFrame: not GEOMETRIC feature processing.");
            }


            if(thingsIShouldBeLookingFor.includes(Algorithm.BOOFCV_SQUARE_FIDUCIAL)) {
                if (DetectionType.STOP == DETECTION_TYPE ) {
                    Log.i(TAG, "onCameraFrame: NOT processing fiducials: DetectionType.STOP == DETECTION_TYPE.");
                } else {
                    if (DetectionType.ONE_IMAGE == DETECTION_TYPE) {
                        DETECTION_TYPE = DetectionType.STOP;
                    }
                        Log.i(TAG, "onCameraFrame: start BoofCV Square Fiducial feature processing.");
                    detectAndEstimate_BoofCV_Fiducial_Binary(robotsDetected, singleDummyRobotId, robotFeatures, landmarkFeatures, logTag, focalLengthCalc, calcImgDim, imageFrameTime);
                        Log.i(TAG, "onCameraFrame: after BoofCV Square Fiducial feature processing.");
                }
            } else {
                Log.i(TAG, "onCameraFrame: NOT BoofCV Square Fiducial feature processing.");
            }

            //--------------------------------------------------------------------------------------

            if (thingsIShouldBeLookingFor.includes(Algorithm.SURF)) {
                ArrayList<VisionTask> visionTasks_SURF = vosTaskSet.visionTasksToExecuteFilter(Algorithm.SURF);
                if (null != visionTasks_SURF && visionTasks_SURF.size() > 0) {
                        Log.i(TAG, "onCameraFrame: before SURF feature processing.");
                    detectAndEstimate_SURF();
                        Log.i(TAG, "onCameraFrame: after SURF feature processing.");
                } else {
                    Log.i(TAG, "onCameraFrame: NOT SURF feature processing.");
                }
            }

            dealWithDetectedFeatures(robotsDetected);
                Log.i(TAG, "onCameraFrame: after reported robot pose ");

        } else {
            Log.i(logTag, "not doing image processing: current_image_bytes_is_not_null="+current_image_bytes_is_not_null+"  &&  thereIsAVisionTaskToExecute="+thereIsAVisionTaskToExecute);
        }

        Log.i(TAG, "onCameraFrame: start HSV segment ");
        if(current_image_bytes_is_not_null  && null != determiningFreeFloorspace) {
            freeSpace(camera);
        }
        Log.i(TAG, "onCameraFrame: after HSV segment ");



        Log.i(TAG, "onCameraFrame: starting OpenCV segment ");
        if( !running_native) {
            Log.i(TAG, "running_native == false: not running native code, not running OpenCV segment ");
        } else {
            imageProcessingNative(focalLengthCalc, calcImgDim);
        } // end  if( running_native)
        Log.i(TAG, "after OpenCV segment ");

        if (screenLocked) {
            blankScreenOutput_OpenCV(logTag);
        }
        smartCameraExtrinsicsCalibrator.imageReceived();

        if(RECORD_DATA_RECORDED && !RECORD_DATA_CONTINUOUS) {
            Log.i(TAG,"onCameraFrame: finishing up: RECORD_x status: before: "+logMessage_recordDataStatus());
            RECORD_DATA = false;
            Log.i(TAG,"onCameraFrame: finishing up: RECORD_x status: after: "+logMessage_recordDataStatus());
        }

        Log.i(TAG,"onCameraFrame: END: cameraNumber="+getCamNum()+": frame="+frameNumber);
        if (displayRgb) {
            renderGUI();
            return matRgb;
        } else {
            return matGray;
        }
    }

    private void incrementFrameProcessedCounter() {
        framesProcessed++;
    }

    /**
     * Returns the current time including the NTP server offset, and optionally checks that TF is
     * available.
     * @return the current time including the NTP server offset
     */
    private Date currentTimeSafe(boolean checkTf) {
        Date imageFrameTime = nowAsDate();
        if(checkTf) {
            try {
                if (null != vosTaskAssignmentSubscriberNode) {
                    Log.i("timecheck:ms before=", Long.toString(imageFrameTime.getTime()));
                    vosTaskAssignmentSubscriberNode.getTfDummy("/map", "/base_link", imageFrameTime);    // test the time offset
                    Log.i("timecheck:ms after=", Long.toString(imageFrameTime.getTime()));
                }
            } catch (Exception e) {
                Log.i("timecheck:exception", "Exception running timecheck: " + e, e);
            }
        }
        return imageFrameTime;
    }

    private void verifyTime() {
        /* TODO - for full NTP impl - several NTP servers, several requests per server, fastest response per server, median filter over time differences across servers - see /mnt/nixbig/downloads/ntp_instacart_truetime_android/truetime-android/library-extension-rx/src/main/java/com/instacart/library/truetime/TrueTimeRx.java */
        //        checkAndLogTimeVsTrueTime("onCameraFrame: time test: system time: ",systemTime);
        //        try {
        //            if(TrueTime.isInitialized()) {
        //                java.util.Date trueTimeDate = TrueTime.now();
        //                org.ros.message.Time timeProviderTime = timeProvider.getCurrentTime();
        //                Log.i(TAG,"onCameraFrame: time test: TrueTime.now() = "+trueTimeDate+" = "+trueTimeDate.getTime() + "ms");
        //                Log.i(TAG,"onCameraFrame: time test: timeProvider.getCurrentTime() = "+timeProviderTime+" = "+( timeProviderTime.totalNsecs() / (1000L*1000L)) + "ms" );
        //                java.util.Date timeProviderTime_as_javaUtilDate = new java.util.Date( ( timeProviderTime.totalNsecs() / (1000L*1000L)) );
        //                checkAndLogTimeVsTrueTime("onCameraFrame: time test: NtpTimeProvider vs TrueTime: ", timeProviderTime_as_javaUtilDate, trueTimeDate);
        //            } else {Log.i(TAG,"onCameraFrame: time test: TrueTime is not initialised.");}
        //        } catch (Exception e) {
        //            Log.e(TAG,"onCameraFrame: exception comparing TrueTime to a dummy request's time: "+e, e);
        //        }
        //        java.util.Date imageFrameTime = systemTime;
    }

    private void incrementFrameCounter() {
        frameNumber++;
    }

//    private void checkAndLogTimeVsTrueTime(String logMsgPrefix_, java.util.Date systemTime) {
//        if(TrueTime.isInitialized()){ java.util.Date realDate = TrueTime.now(); Log.i(TAG,logMsgPrefix_+": date from TrueTime.now() - arg[["+systemTime+"]] = "+(realDate.getTime()-systemTime.getTime())+"milliseconds");}
//        else{Log.i(TAG,logMsgPrefix_+": TrueTime is not initialised.");}
//    }

//    private void checkAndLogTimeVsTrueTime(String logMsgPrefix_, java.util.Date systemTime , java.util.Date trueTime) {
//        Log.i(TAG,logMsgPrefix_+": date from arg[["+trueTime+"]] - arg[["+systemTime+"]] = "+(trueTime.getTime()-systemTime.getTime())+"milliseconds");
//    }

    private void freeSpace(Camera camera) {
        Log.i(TAG, "onCameraFrame: HSV segment: current_image_bytes_is_not_null ");

        Log.i(TAG, "onCameraFrame: HSV segment: before convertPreviewBoofCVHsv");
        convertPreviewBoofCVHsv(last_frame_bytes(), camera);
        Log.i(TAG, "onCameraFrame: HSV segment: after convertPreviewBoofCVHsv");


//            Planar<GrayF32> hs = imageHsv.partialSpectrum(0,1);
//            // The number of bins is an important parameter.  Try adjusting it
//            Histogram_F64 histogram = new Histogram_F64(25,25);
//            histogram.setRange(0, 0.0, 1.0); // range of hue is from 0 to 2PI
//            histogram.setRange(1, 0.0, 1.0); // range of saturation is from 0 to 1
//
//            // Compute the histogram
//            GHistogramFeatureOps.histogram(hs,histogram);
//
//            UtilFeature.normalizeL2(histogram); // normalize so that image size doesn't matter


        // Extract hue and saturation bands which are independent of intensity
        GrayF32 H = imageHsv.getBand(0);
        GrayF32 S = imageHsv.getBand(1);
        Log.i(TAG, "onCameraFrame: HSV segment: saturation band height = " + S.getHeight() + ", saturation band width = " + S.getWidth() + " ");

//            // Adjust the relative importance of Hue and Saturation.
//            // Hue has a range of 0 to 2*PI and Saturation from 0 to 1.
        float adjustUnits = (float) (Math.PI / 2.0);
        // Euclidean distance squared threshold for deciding which pixels are members of the selected set
        float maxDist2 = 0.4f * 0.4f;
        float hue = 0.12f;
        float saturation = 0.15f;
        double[] red = new double[]{255d, 0d, 0d, 255d};
        double[] green = new double[]{0d, 255d, 0d, 255d};
        double[] greenBlue = new double[]{0d, 0d, 255d, 255d};
        double[] free_space_white = new double[]{255d, 255d, 255d, 125d};
        double[] black = new double[]{0d, 0d, 0d, 255d};
        int i_ = 0;
        int numberOfFilters = 0;
        if (determiningFreeFloorspace.contains("HSV")) {
            Log.i(TAG, "onCameraFrame: HSV segment: determiningFreeFloorspace with colour/HS(V).");
            numberOfFilters++;
            for (int y = 0; y < imageHsv.height; y++) {            // https://boofcv.org/index.php?title=Example_Color_Segmentation
                for (int x = 0; x < imageHsv.width; x++) {
                    i_++;
//                    if(0== i_ % 100) {
//                        Log.i(TAG, "onCameraFrame: HSV segment: "+i_+"th iteration: x="+x+", y="+y);
//                    }
                    // Hue is an angle in radians, so simple subtraction doesn't work
                    float h_pixel = H.unsafe_get(x, y);                      // range of 0.0 to 2.0*PI
                    float s_pixel = S.unsafe_get(x, y);                      // range of 0.0 to 1.0
//                    float dh      = UtilAngle.dist(h_pixel,hue);            // range of 0.0 to 2.0*PI
//                    float ds      = (s_pixel-saturation)*adjustUnits;       // range of 0.0 to 2.0*PI
//
                    // this distance measure is a bit naive, but good enough for to demonstrate the concept
//                    float dist2 = dh*dh + ds*ds;
//                    if( dist2 <= maxDist2 ) {
                    if (  // varies a lot with the white balance
                            (h_pixel >= 2.0f    // 2.26893f
                                    &&
                                    h_pixel <= 4.6f)  // 4.39823f   )
                                    &
                                    s_pixel <= 0.35f
                            ) {
//                        output.setRGB(x,y,image.getRGB(x,y));
                        hsvMatch[x][y] = 1;                     //  hsvMatch = new boolean[image.width][image.height]
                        matRgb.put(y, x, green);
                    } else {
                        hsvMatch[x][y] = 0;                     //  hsvMatch = new boolean[image.width][image.height]
                        matRgb.put(y, x, red);
                    }
                }
            }
            for (int y = 0 + 3; y < imageHsv.height - 3; y = y + 7) {            // https://boofcv.org/index.php?title=Example_Color_Segmentation
                for (int x = 0 + 3; x < imageHsv.width - 3; x = x + 7) {

                    int sum_ = 0;
                    for (int y2 = y - 3; y2 <= y + 3; y2++) {
                        for (int x2 = x - 3; x2 <= x + 3; x2++) {
                            sum_ += hsvMatch[x2][y2];
                        }
                    }
                    if (sum_ > 25) { // over 50% good
                        for (int y2 = y - 3; y2 <= y + 3; y2++) {
                            for (int x2 = x - 3; x2 <= x + 3; x2++) {
                                matRgb.put(y2, x2, greenBlue);                  // TODO - boolean output for the block
                                combinedOutput[x2][y2]++;
                                if(free_hist[x2][y2] < free_hist_window_length) {
                                    free_hist[x2][y2]++;
                                }
                            }
                        }
                    } else {
                        for (int y2 = y - 3; y2 <= y + 3; y2++) {
                            for (int x2 = x - 3; x2 <= x + 3; x2++) {
                                // TODO - output for the block
                                if(free_hist[x2][y2]>0) {
                                    free_hist[x2][y2]--;
                                }
                            }
                        }
                    }
                }
            }

            Log.i(TAG, "onCameraFrame: HSV segment: end determiningFreeFloorspace with colour/HS(V).");
        }

        if (determiningFreeFloorspace.contains("Texture")) {
            Log.i(TAG, "onCameraFrame: HSV segment: determiningFreeFloorspace with texture.");
            numberOfFilters++;
            //--- gray level run-length ------------------------------------------------------------
            GrayF32 V = imageHsv.getBand(2);
            GrayF32 gradient_img = V.createNew(imageHsv.width, imageHsv.height);
            GrayF32 run_count_img = V.createNew(imageHsv.width, imageHsv.height);
            int run_count_bins[] = new int[imageHsv.width]; // could have a run all the same intensity, e.g. facing a blank wall, or camera covered.

            float max_intensity = 254.0f;
            float intensity_left = 0.0f;
            float intensity_previous = 0.0f;
            float intensity_right;
            float intensity = 0.0f;
            for (int y = 0; y < imageHsv.height; y++) {            // https://boofcv.org/index.php?title=Example_Color_Segmentation
                for (int x = 0; x < imageHsv.width; x++) {
                    intensity_previous = intensity;
                    if (x > 0) {
                        intensity_left = intensity_previous;
                        intensity = V.unsafe_get(x, y);
                        if (intensity_left - intensity > (max_intensity / 20.0f)) {
                            gradient_img.set(x, y, max_intensity);
                        }  // simplified from TextureFilter
                    }
                    //if(x < imageHsv.width-1) { intensity_right = V.unsafe_get(x+1,y);}
                }
                // run back over the row, counting runs of the same(ish) gradient

                int run_count = 0;
                float last_gradient = 0.0f;
                float param_run_count_tolerance = 0.1f;
                boolean param_allow_for_slow_gradient = false;
                for (int x = 0; x < imageHsv.width; x++) {

                    if (last_gradient - param_run_count_tolerance < gradient_img.unsafe_get(x, y) && gradient_img.unsafe_get(x, y) < last_gradient + param_run_count_tolerance) {
                        run_count++;
                    } else {
                        int run_count_was = run_count;
                        run_count_bins[run_count]++;                        // increment the bin               up to the current pixel
                        run_count = 0;                                        // reset run_count                 at the current pixel
                        last_gradient = gradient_img.unsafe_get(x, y);     // reset the gradient to the value at the current pixel

                        int indexScaled_backward = x;
                        for (int x_neg = x - 1; x_neg > 0 && x_neg >= ((x - 1) - run_count_was); x_neg--) {
                            int param_run_count_upper_cutoff = 20;                                                //  TODO - by hand fiddling !
                            if (run_count_was >= param_run_count_upper_cutoff) {                                  //  TODO - by hand fiddling !
                                run_count_img.set(x_neg, y, 254.0f);
                                runlengthMatch[x_neg][y] = 1;
                            } else {
                                ;
                                run_count_img.set(x_neg, y, ((float) run_count_was / (float) param_run_count_upper_cutoff) * 254.0f);  // approximate: will break if any run is over 254 long, but OK for now
                                runlengthMatch[x_neg][y] = 0;
                            }
                        }
                    }
                }
            } // end of row for runlengthMatch
            for (int y = 0 + 3; y < imageHsv.height - 3; y = y + 7) {            // https://boofcv.org/index.php?title=Example_Color_Segmentation
                for (int x = 0 + 3; x < imageHsv.width - 3; x = x + 7) {

                    int sum_ = 0;
                    for (int y2 = y - 3; y2 <= y + 3; y2++) {
                        for (int x2 = x - 3; x2 <= x + 3; x2++) {
                            sum_ += runlengthMatch[x2][y2];
                        }
                    }
                    if (sum_ < 25) { // under XYZ                                               // TODO - PARAMETER - should look at the runlengths in bins
                        for (int y2 = y - 3; y2 <= y + 3; y2++) {
                            for (int x2 = x - 3; x2 <= x + 3; x2++) {
                                matRgb.put(y2, x2, greenBlue);                                  // TODO - boolean output for the block
                                combinedOutput[x2][y2]++;
                            }
                        }
                    }

                }
            }
            Log.i(TAG, "onCameraFrame: HSV segment: end determiningFreeFloorspace with texture.");
        }

        // combine boolean outputs for the block: GOOD && GOOD == GOOD == FREE
        if (numberOfFilters > 0) {
            Log.i(TAG, "onCameraFrame: HSV segment: determiningFreeFloorspace checking combined filter outputs.");
            // TODO - do this blockwise and use a hashmark pattern ?
            for (int y = 0; y < imageHsv.height; y++) {            // https://boofcv.org/index.php?title=Example_Color_Segmentation
                for (int x = 0; x < imageHsv.width; x++) {
                    if (combinedOutput[x][y] >= numberOfFilters) {
                        matRgb.put(y, x, free_space_white);                                  // TODO - boolean output for the block
                    }
                }
            }
            Log.i(TAG, "onCameraFrame: HSV segment: end determiningFreeFloorspace checking combined filter outputs.");


            if (determiningFreeFloorspace.contains("display_projected")){
                Log.i(TAG, "onCameraFrame: HSV segment: determiningFreeFloorspace display_projected.");
                // Display a projection onto an arbitrary 10x10 world, plan view, with -5<=y<=5 and 0<=x<=10, with camera at y=0 x=0, and camera x = world x, and camera y = world y, and camera z = world z + camera pose z.
//                    for (int y = 0; y < imageHsv.height; y++) {            // https://boofcv.org/index.php?title=Example_Color_Segmentation
//                        for (int x = 0; x < imageHsv.width; x++) {
//                            matRgb.put(y, x, black);                                  // TODO - boolean output for the block
//                        }
//                    }
                for (int y = imageHsv.height/2; y < imageHsv.height; y++) {            // todo: don't bother about the upper half of the image for now: will be off the floorplain for now
                    for (int x = 0; x < imageHsv.width; x++) {
                        if (combinedOutput[x][y] >= numberOfFilters) {
                            projectOntoWorldFreeSpace(x, y, imageHsv.width, imageHsv.height, position[2], matRgb, free_space_gold, free_space_projected_purple);  // TODO - paint onto the image, clipping at the image edges
                        }
                    }
                }
            }
            Log.i(TAG, "onCameraFrame: HSV segment: end determiningFreeFloorspace display_projected.");


            if (determiningFreeFloorspace.contains("display_hist_projected")){
                Log.i(TAG, "onCameraFrame: HSV segment: determiningFreeFloorspace display_projected.");
                for (int y = imageHsv.height/2; y < imageHsv.height; y++) {            // todo: don't bother about the upper half of the image for now: will be off the floorplain for now
                    for (int x = 0; x < imageHsv.width; x++) {
                        if (free_hist[x][y] >= free_hist_window_average) {
                            free_space_frozen[x][y] = 1;
                            projectOntoWorldFreeSpace(x, y, imageHsv.width, imageHsv.height, position[2], matRgb, free_space_darkGreen, free_space_projected_aqua);  // TODO - paint onto the image, clipping at the image edges
                        }
                        if(free_space_frozen[x][y] > 0) {
                            if(combinedOutput[x][y] >= numberOfFilters) {
                                projectOntoWorldFreeSpace(x, y, imageHsv.width, imageHsv.height, position[2], matRgb, free_space_paleGreen, free_space_projected_darkCyan);
                            } else {
                                projectOntoWorldFreeSpace(x, y, imageHsv.width, imageHsv.height, position[2], matRgb, free_space_magenta, free_space_projected_mediumOrchid);
                            }
                        }
                    }
                }
            }
            Log.i(TAG, "onCameraFrame: HSV segment: end determiningFreeFloorspace display_projected_hist.");


        } else {
            Log.i(TAG, "onCameraFrame: HSV segment: determiningFreeFloorspace NOT checking combined filter outputs : numberOfFilters="+numberOfFilters);
        }
    }

    private void detectAndEstimate_SURF() {
        try {
            String[] surfFeatureDescriptors = Hardcoding.testSurfFeatureDescriptors();
            String[] imageBfeatureString = surfFeatureDescriptors;
            Class imageTypeGrayF32 = GrayF32.class;   //  Class imageType = GrayF32.class;
            Class imageRgbType = Planar.class;
            AssociatePoints app = setupAssociatePointsVariables(imageTypeGrayF32, imageRgbType);
            AssociatePoints.ReturnValue associations = app.associateSurfAndString(image, imageBfeatureString);    // -- SURF !!!

            FastQueue<AssociatedIndex> matches = associations.associate_getMatches;
            Log.i(TAG, "onCameraFrame: SURF feature: matches.size=" + matches.size());
            List<Point2D_F64> leftPts = associations.pointsA;
            List<Point2D_F64> rightPts = associations.pointsB;
//                // from AssociationPanel
//                public void drawPoints( List<Point2D_F64> leftPts , List<Point2D_F64> rightPts,
//                        FastQueue< AssociatedIndex > matches ) {
            int assocLeft[], assocRight[];    // which features are associated with each other
            List<Point2D_F64> allLeft = new ArrayList<>();
            List<Point2D_F64> allRight = new ArrayList<>();
            assocLeft = new int[matches.size()];
            assocRight = new int[matches.size()];
            for (int i = 0; i < matches.size(); i++) {
                Log.i(TAG, "onCameraFrame: SURF feature: matches[" + i + "]");
                AssociatedIndex a = matches.get(i);
                allLeft.add(leftPts.get(a.src));
                allRight.add(rightPts.get(a.dst));
                assocLeft[i] = i;
                assocRight[i] = i;
            }
            for (int i = 0; i < assocLeft.length; i++) {
                if (assocLeft[i] == -1) {
                    continue;
                }
                Log.i(TAG, "onCameraFrame: SURF feature: ");
                Point2D_F64 l = leftPts.get(i);
                Point2D_F64 r = rightPts.get(assocLeft[i]);
                //            Color color = colors[i];
                //            drawAssociation(g2, scaleLeft,scaleRight,rightX, l, r, color);
//                        drawSurfFeatureLocus(l.getX(),l.getY())
//                        displayTagCentre_OpenCV((int)locationPixel.x, (int)locationPixel.y, fm);
                displaySurf_OpenCV((int) l.getX(), (int) l.getY());
            }
//                }

        } catch (Exception e) {
            Log.e(TAG, "onCameraFrame: exception running SURF features: ", e);
            e.printStackTrace();
        }
        Log.i(TAG, "onCameraFrame: after SURF feature processing.");
    }


    boolean FIDUCIAL_MASK_WITH_HSV = false;

    public void manage_maskFiducialWithHSV(boolean maskFiducialWithHSV_) {
        FIDUCIAL_MASK_WITH_HSV = maskFiducialWithHSV_;
        Log.i(TAG,"manage_maskFiducialWithHSV: FIDUCIAL_MASK_WITH_HSV = "+FIDUCIAL_MASK_WITH_HSV);
    }

    private void detectAndEstimate_BoofCV_Fiducial_Binary(HashMap<RobotId, List<DetectedTag>> robotsDetected_, RobotId singleDummyRobotId_, List<DetectedTag> robotFeatures, List<DetectedTag> landmarkFeatures, String logTag, FocalLengthCalculator focalLengthCalc, CalcImageDimensions calcImgDim, java.util.Date imageFrameTime) {
        Log.i(logTag, "detectAndEstimate_BoofCV_Fiducial_Binary: start: matGray.width()="+matGray.width()+", matGray.height()="+matGray.height()); //// TODO - timing here  c[camera_num]-f[frameprocessed]
        try {
            FiducialDetector<GrayF32> detector = FactoryFiducial.squareBinary(
                    new ConfigFiducialBinary(Hardcoding.BOOFCV_MARKER_SIZE_M), ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 10), GrayF32.class);  // tag size,  type,  ?'radius'?
            //        detector.setLensDistortion(lensDistortion);

            // /mnt/nixbig/ownCloud/project_AA1__2_extrinsics_calibration/project_AA1_2_extrinsics__phone_data_recording__copied_bc/VOS_data_2018_01_21_16_51_11_calibration_pattern_Galaxy3_home/intrinsics_matlab_3radial_2tangental_0skew.txt
            CameraPinholeRadial cameraPinholeRadial = new CameraPinholeRadial(
                    322.9596901156589, 323.8523693059909, //fx,fy,
                    0.0,                                    // skew,
                    176.8267919600727, 146.7681514313797, // cx, cy,
                    calcImgDim.getWidth(), calcImgDim.getHeight());
            cameraPinholeRadial.fsetRadial(0.004180016841640, 0.136452931271259, -0.638647134308425);
            cameraPinholeRadial.fsetTangental(-0.001666231998527, -0.00008160213039217031);
            LensDistortionRadialTangential lensDistortionRadialTangential = new LensDistortionRadialTangential(cameraPinholeRadial);
            detector.setLensDistortion(lensDistortionRadialTangential);

            CameraPinhole pinholeModel = new CameraPinhole(
                    FocalLengthCalculator.getFocal_length_in_pixels_x(matGray.width()),
                    FocalLengthCalculator.getFocal_length_in_pixels_y(matGray.height()),
                    calcImgDim.getSkew(),
                    calcImgDim.getPx_pixels(), calcImgDim.getPy_pixels(),
                    calcImgDim.getWidth(), calcImgDim.getHeight());
            LensDistortionNarrowFOV pinholeDistort = new LensDistortionPinhole(pinholeModel);
            detector.setLensDistortion(pinholeDistort);                                             // TODO - do BoofCV calibration - but assume perfect pinhole camera for now


            boofcv.struct.image.GrayF32 image2 = image;
            if(FIDUCIAL_MASK_WITH_HSV) {
                image2 = image.clone();
                convertPreviewBoofCVHsv(last_frame_bytes(), null);
                GrayF32 V = imageHsv.getBand(2);
                GrayF32 S = imageHsv.getBand(1);
                // checks H.getHeight() H.getWidth()

                for (int y = 0; y < imageHsv.height; y++) {            // https://boofcv.org/index.php?title=Example_Color_Segmentation
                    for (int x = 0; x < imageHsv.width; x++) {
//                      if( V.get(x,y) > 0.3 && S.get(x,y) > 0.2 ) {   // bright enough not to be black, colourful enough not to be white
                      if( S.get(x,y) > 0.4 ) {   // bright enough not to be black, colourful enough not to be white
                            image2.set(x,y,127.0f);
                            double[] colour = new double[] {127d,127d,127d,255d};
                            matRgb.put(y, x, colour);
                        }
                    }
                }
            }


                Log.i(logTag, "start detector.detect(image);"); //// TODO - timing here  c[camera_num]-f[frameprocessed]
            detector.detect(image2);
                Log.i(TAG, "onCameraFrame: found " + detector.totalFound() + " tags via BoofCV");
                Log.i(logTag, "finished detector.detect(image);"); //// TODO - timing here  c[camera_num]-f[frameprocessed]

            boolean smartCameraExtrinsicsCalibrator_first_notification_this_frame = true;
            final boolean DETECT_ALL_MARKERS_IN_FRAME = true;
            boolean hasDetectedMarker = false;
            // see https://boofcv.org/index.php?title=Example_Fiducial_Square_Image
            for (int detectionOrder_ = 0; detectionOrder_ < detector.totalFound(); detectionOrder_++) {
                    //// TODO - timing here  c[camera_num]-f[frameprocessed]-detectionOrder_[iteration]
                    String logTagIteration = logTag + "-detectionOrder_" + detectionOrder_;
                    Log.i(logTagIteration, "start");

                int tag_id = (int) detector.getId(detectionOrder_);

                if (checkAndDrawIfTagIsInvalid(detector, detectionOrder_, tag_id)) { continue; }

                VisionTask visionTask = vosTaskSet.visionTaskToExecute(Algorithm.BOOFCV_SQUARE_FIDUCIAL,tag_id);     // NOTE: BoofCV square fiducial is an open-ended algorithm which we then narrow down to the descriptors in question, whereas e.g. Kaess includes/excludes within the algorithm.
                if (checkAndDrawIfTagHasNoVisiontask(detector, detectionOrder_, visionTask)) { continue; }

                    //// TODO - timing here  c[camera_num]-f[frameprocessed]-detectionOrder_[iteration]-t[tagid]

                visionTask.executedAndDetected();      //
                drawMarkeLocationOnDisplay_BoofCV(detector, detectionOrder_, visionTask);

                List<VisionTask> visionTaskListToExecute = vosTaskSet.visionTaskListToExecute(Algorithm.BOOFCV_SQUARE_FIDUCIAL,tag_id);
                hasDetectedMarker = true;

                    String logTagTag = logTagIteration + "-t" + tag_id;
                    Log.i(logTagTag, "onCameraFrame: finished checking tag_id");

                logAnyMessageFromTheDetector(detector, detectionOrder_);

                if (detector.is3D()) {
                    Log.i(logTagTag, "onCameraFrame: start detector.getFiducialToCamera(detectionOrder_, targetToSensor_boofcvFrame);");
                    PixelPosition pixelPosition = pixelPositionOfTag(detector, detectionOrder_);

                    Se3_F64 targetToSensor_boofcvFrame = new Se3_F64();
                    detector.getFiducialToCamera(detectionOrder_, targetToSensor_boofcvFrame);


                    boolean new__detect_tags_and_report_them_to_VOS_Server_detect_process = false;
                    if (new__detect_tags_and_report_them_to_VOS_Server_detect_process) {
                        //convert__tagToSensor_BodyFrame__to__sensorToTag_BodyFrame
                        Se3_F64 StoT_Se3 = targetToSensor_boofcvFrame.invert(null);

                        // convert_BoofCVTag_luf_CoordinateSystem_to_robot_flu_CoordinateSystem:{ // convert from LUF to FLU
                            Vector3D_F64    StoT_luf_t = StoT_Se3.getTranslation();
                            Quaternion_F64  StoT_luf_q = transform_to_quaternion_boofcv(StoT_Se3);
                            Vector3D_F64    StoT_flu_t = new Vector3D_F64(                  StoT_luf_t.z , StoT_luf_t.x , StoT_luf_t.y );  // Vector3D_F64( double x, double y, double z )
                            Quaternion_F64  StoT_flu_q = new Quaternion_F64( StoT_luf_q.w , StoT_luf_q.z , StoT_luf_q.x , StoT_luf_q.y );  // Quaternion_F64(double w, double x, double y, double z)

                        ask_for_the_robots_pose_via_cache:{
                            ask_for_robot_pose:{
                                // this can take time - --> run asynch
                                // only know which robot to ask after we see it in-frame for the first -
                                // mind you, in the next frame we'll know, so we can probably ask for the same set of robots again on the next frame; if we don't use it - meh; if we see a new robot, we can get that pose async

                                // async_ask_for_robot_pose(time_of_image)
                                    // this runs in another thread so that the rest can run on - the callback hooks the observation up to the robot pose, and calls to estimate_camera_pose(current_observation_set)
                            }
                        }
                    } else {
                        // detect tags and report them to VOS Server detect_ process
                        // don't remember why we skip here if this returns false
                        BoofCVDetectorAndReporter boofcvDetectorAndReporter = new BoofCVDetectorAndReporter(robotsDetected_, singleDummyRobotId_, landmarkFeatures, imageFrameTime, detector, smartCameraExtrinsicsCalibrator_first_notification_this_frame, DETECT_ALL_MARKERS_IN_FRAME, detectionOrder_, tag_id, visionTask, visionTaskListToExecute, logTagTag, targetToSensor_boofcvFrame, pixelPosition);
                        boofcvDetectorAndReporter.invoke();
                        if (boofcvDetectorAndReporter.is()) { continue; }
                    }

                    //--- record image and detection details to file -------------------------------
                    // TODO - should not just be part of this algorithm; should be part of the general data recording / logging process
                    recordTheImageOrNot(imageFrameTime, logTagTag, hasDetectedMarker);

                } else {  // 3D info not available for tag/marker
                    drawMarkeLocationOnDisplay_BoofCV_no3dData(detector, detectionOrder_);
                }

                Point2D_F64 locationPixel = new Point2D_F64();
                detector.getImageLocation(detectionOrder_,locationPixel);
//                PixelPosition pixelPosition = new PixelPosition(locationPixel.getX(),locationPixel.getY(), matGray.size().width, matGray.size().height);
//                if(smartCameraExtrinsicsCalibrator_first_notification_this_frame) {
//                    smartCameraExtrinsicsCalibrator_first_notification_this_frame = false;
//                    this.smartCameraExtrinsicsCalibrator.detectedInImage(pixelPosition,transformOfFeatureInVisualModel_inv);
//                }

            }
            updateTrackingData(robotFeatures);
            if (FOUR_POINTS_REQUIRED_FOR_PNP <= landmarkFeatures.size()) {
                Se3_F64 cameraPose = updatePoseEstimate(landmarkFeatures);
                int numElementsInRot = cameraPose.R.getNumElements();
                Vector3D_F64 translation = cameraPose.T;
                Log.i(TAG, "onCameraFrame: cameraPose: " + /*" rotation numElements=" + numElementsInRot + ", rotation=" + cameraPose.R.toString() + */", translation =" + translation.toString());
            } else {
                Log.i(TAG, "onCameraFrame: cameraPose: not enough landmarks detected to estimate camera pose.");
            }

            Log.i(TAG, "onCameraFrame: after processing for " + detector.totalFound() + " tags found via BoofCV");

        } catch (Exception e) {
            Log.e(TAG, "onCameraFrame: exception running BoofCV fiducial: ", e);
            e.printStackTrace();
        }
    }

    private void recordTheImageOrNot(Date imageFrameTime, String logTagTag, boolean override_hasDetectedMarker) {
        if(RECORD_DATA || override_hasDetectedMarker) {
            Log.i(logTagTag, "detectAndEstimate_BoofCV_Fiducial: RECORD_DATA: recording data: start: "+logMessage_recordDataStatus());
            printlnToFile("matGray_size: { width: " + matGray.width() + ", height:" + matGray.height() + " }", imageFrameTime);
            printlnToFile("frame: " + Long.toString(frameNumber), imageFrameTime);
            saveImageDataToFile(matGray, imageFrameTime, "grey");
            saveImageDataToFile(matRgb, imageFrameTime, "RGB");
            RECORD_DATA_RECORDED = true;
            Log.i(logTagTag, "detectAndEstimate_BoofCV_Fiducial: RECORD_DATA: recording data: end: "+logMessage_recordDataStatus());
        } else {
            Log.i(logTagTag, "detectAndEstimate_BoofCV_Fiducial: RECORD_DATA: NOT recording data: "+logMessage_recordDataStatus());
        }
    }

    @NonNull
    private PixelPosition pixelPositionOfTag(FiducialDetector<GrayF32> detector, int detectionOrder_) {
        Point2D_F64 locationPixel = new Point2D_F64();
        detector.getImageLocation(detectionOrder_,locationPixel);
        return new PixelPosition(locationPixel.getX(),locationPixel.getY(), matGray.size().width, matGray.size().height);
    }


    private FileWriter openFileForData(Date imageFrameTime_, String fileName_) {
        try {
            File documentsDirectory = configureDirectoryForData(imageFrameTime_);
            File dataFile = new File(documentsDirectory, fileName_);
            dataFile.setReadable(true, true);
            Log.i("openFileForData","openFileForData(\""+fileName_+"\"): file is ready at "+documentsDirectory.getAbsoluteFile()+"  "+fileName_);
            return new FileWriter(dataFile);
        } catch (IOException e) {
            Log.e("openFileForData","openFileForData(\""+fileName_+"\"): could not open the file.", e);
        }
        return null;
    }


    private File configureDirectoryForData(Date imageFrameTime_) {
        if (null == dataOutputDirectory) {
            dataOutputDirectory = dirInExternalStorage(
                    VOS_DATA_DIRECTORY_NAME + "_" + filenameDateFormat.format(imageFrameTime_));
        }
        return dataOutputDirectory;
    }

    /**
     * Idempotent; if detectionDataOutputFile is already configured, this has no effect.
     * @param imageFrameTime_ the time to use to name the data output file: the first iteration / image capture time.
     */
    private FileWriter configureFileForData(Date imageFrameTime_) {
        if (null == detectionDataOutputFile) {
            openDataOutputFile(imageFrameTime_);
        }
        return detectionDataOutputFile;
    }

    public void resetDataOutputDirectory() {
        Date time_now = DateAndTime.nowAsDate();
        Log.i(TAG, "resetDataOutputDirectory(): "+time_now);
        configureDirectoryForDataReconfigureImmediate(time_now);
        configureFileForDataReconfigureImmediate(time_now);
    }

    private File configureDirectoryForDataReconfigureImmediate(Date imageFrameTime_) {
        dataOutputDirectory = dirInExternalStorage(
            VOS_DATA_DIRECTORY_NAME + "_" + filenameDateFormat.format(imageFrameTime_));
        return dataOutputDirectory;
    }

    /**
     * Idempotent; if detectionDataOutputFile is already configured, this has no effect.
     * @param imageFrameTime_ the time to use to name the data output file: the first iteration / image capture time.
     */
    private FileWriter configureFileForDataReconfigureImmediate(Date imageFrameTime_) {
        if (null != detectionDataOutputFile) {
            closeDataOutputFile();
        }
        openDataOutputFile(imageFrameTime_);
        return detectionDataOutputFile;
    }

    private void openDataOutputFile(Date imageFrameTime_) {
        String detectionDataOutputFileName = filenameDateFormat.format(imageFrameTime_);
        detectionDataOutputFileName = detectionDataOutputFileName + "_savedData.txt";
        detectionDataOutputFile = openFileForData(imageFrameTime_, detectionDataOutputFileName);
    }

//    private boolean configureDirectoryForImages(String directoryName_) {
//        File dir = dirInExternalStorage(directoryName_);
//        boolean success = true;
//        if (!dir.exists()) {
//            success = dir.mkdirs();
//        }
//        return success;
//    }

    @NonNull
    private File dirInExternalStorage(String directoryName_) {
        File dir = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS), directoryName_);
        if(!dir.exists()) {
            if(!dir.mkdirs()) {
                Log.e("dirInExternalStorage","dirInExternalStorage directory "+dir.getAbsolutePath()+" does not exist and cannot be created.");
            }
            dir.setReadable(true,true);
        }
        return dir;
    }

    private File fileInExternalStorage(Date imageFrameTime_, String fileName_) {
        File dir  = configureDirectoryForData(imageFrameTime_);
        File file = new File(dir, fileName_);
        return file;
    }


    /** Uses the datetime of image capture: the data saved relates to the image, not to the datetime of the statement execution.
     * @param text_ text to save to the file.
     * @param imageFrameTime_ : the datetime of image capture: the data saved relates to the image, not to the datetime of the statement execution.
     */
    public void printlnToFile(String text_, Date imageFrameTime_) {
        configureFileForData(imageFrameTime_);
        try {
            detectionDataOutputFile.append(datafileLogDateFormat.format(imageFrameTime_));
            detectionDataOutputFile.append(" : ");
            detectionDataOutputFile.append(text_);
            detectionDataOutputFile.append("\n");
            try { detectionDataOutputFile.flush(); }
            catch (IOException e) { Log.e("printlnToFile","printlnToFile: could not flush"); }
        }
        catch (IOException e) { Log.e("printlnToFile","printlnToFile: could not append to file", e); }
    }

    private void printToFile(String text_, Date imageFrameTime_) {
        configureFileForData(imageFrameTime_);
        try {
            detectionDataOutputFile.append(" : ");
            detectionDataOutputFile.append(text_);
            try { detectionDataOutputFile.flush(); }
            catch (IOException e) { Log.e("printlnToFile","printlnToFile: could not flush"); }
        }
        catch (IOException e) { Log.e("printlnToFile","printlnToFile: could not append to file", e); }
    }


    private void saveImageDataToFile(Mat imageToSave_, Date imageFrameTime_, String prefix_) {
        Log.i("saveImageDataToFile", "MainActivity: saveImageDataToFile: start:"+logMessage_recordDataStatus());
//        if( !configureDirectoryForImages(VOS_DATA_DIRECTORY_NAME)) {
//            Log.e("saveImageDataToFile", "MainActivity: saveImageDataToFile: could not configure the image saving directory.");
//            return;
//        }
        if(null==prefix_) {prefix_="";}
        File dest = fileInExternalStorage(
                imageFrameTime_,
                filenameDateFormat.format(imageFrameTime_) + "_" + prefix_ + "_image.png");
        FileOutputStream out = null;
        Log.i("saveImageDataToFile", "MainActivity: saveImageDataToFile: destination file = "+dest.getAbsolutePath());
        //-------------------------------------------------------
        //  http://answers.opencv.org/question/68206/saving-an-image-using-opencv-android/
        Log.i("saveImageDataToFile", "MainActivity: saveImageDataToFile: convert to bitmap: start.");
        Bitmap bmp = null;
        try {
            bmp = Bitmap.createBitmap(imageToSave_.cols(), imageToSave_.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(imageToSave_, bmp);
        } catch (CvException e) {
            Log.e("saveImageDataToFile", "MainActivity: saveImageDataToFile: convert to bitmap: exception: "+e.getMessage(), e);
            return;
        }
        Log.i("saveImageDataToFile", "MainActivity: saveImageDataToFile: convert to bitmap: end.");
        //-------------------------------------------------------
        try {
            Log.i("saveImageDataToFile", "MainActivity: saveImageDataToFile: open FileOutputStream: start.");
            out = new FileOutputStream(dest);
            Log.i("saveImageDataToFile", "MainActivity: saveImageDataToFile: open FileOutputStream: end.");
            Log.i("saveImageDataToFile", "MainActivity: saveImageDataToFile: bitmap compress to PNG and output: start.");
            bmp.compress(Bitmap.CompressFormat.PNG, 100, out); // bmp is your Bitmap instance
            // PNG is a lossless format, the compression factor (100) is ignored
            Log.i("saveImageDataToFile", "MainActivity: saveImageDataToFile: bitmap compress to PNG and output: end.");
        } catch (Exception e) {
            e.printStackTrace();
            Log.e("saveImageDataToFile", "MainActivity: saveImageDataToFile: could not output - exception: "+e.getMessage(), e);
        } finally {
            try {
                if (out != null) {
                    out.close();
                    Log.i("saveImageDataToFile", "MainActivity: saveImageDataToFile: out FileOutputStream closed.");
                }
            } catch (IOException e) {
                Log.e("saveImageDataToFile", "MainActivity: saveImageDataToFile: could not close out FileOutputStream: "+e.getMessage(), e);
                e.printStackTrace();
            }
        }
    Log.i("saveImageDataToFile", "MainActivity: saveImageDataToFile: end: "+logMessage_recordDataStatus());
    }

    private void logAnyMessageFromTheDetector(FiducialDetector<GrayF32> detector, int detectionOrder_) {
        if (detector.hasMessage()) {
            System.out.println("onCameraFrame: Message   = " + detector.getMessage(detectionOrder_));
        }
    }

    private boolean checkAndDrawIfTagHasNoVisiontask(FiducialDetector<GrayF32> detector, int detectionOrder_, VisionTask visionTask) {
        if (null == visionTask) {
            drawMarkeLocationOnDisplay_BoofCV_invalidTagId(detector, detectionOrder_);
            return true;
        }
        return false;
    }

    private boolean checkAndDrawIfTagIsInvalid(FiducialDetector<GrayF32> detector, int detectionOrder_, int tag_id) {
        MarkerIdFormatValidator markerIdFormatValidator = new MarkerIdFormatValidator(detector, detectionOrder_, tag_id);
        if (!markerIdFormatValidator.isValid()) {
            drawMarkeLocationOnDisplay_BoofCV_invalidTagId(detector, detectionOrder_);
            return true;
        }
        return false;
    }

    private void askForPose_then_recordDetection(Date imageFrameTime, Se3_F64 transformOfFeatureInVisualModel, PixelPosition pixelPosition) {
        FrameTransform frameTransform = null;
        try {
            Log.i(TAG,"detectAndEstimate..: before askRobotForItsPoseFrame: DateAndTime.diffNs(imageFrameTime, frameTransform.getTime()) = "+ DateAndTime.diffNs(imageFrameTime, frameTransform.getTime()));
            long tic = nowAsDate().getTime();
            Log.i(TAG,"detectAndEstimate..: before askRobotForItsPoseFrame("+imageFrameTime.getTime()+"ms): time now="+tic+"ms");
            frameTransform = smartCameraExtrinsicsCalibrator.askRobotForItsPoseFrame(imageFrameTime);
            long toc = nowAsDate().getTime();
            Log.i(TAG,"detectAndEstimate..: after askRobotForItsPoseFrame("+imageFrameTime.getTime()+"ms): time now="+toc+"ms: took "+(toc-tic)+"ms");
            Log.i(TAG,"detectAndEstimate..: after askRobotForItsPoseFrame("+imageFrameTime.getTime()+"ms): frameTransform="+frameTransform);
//            smartCameraExtrinsicsCalibrator.recordRobotPose("", frameTransform);
        }
        catch (Exception e) { Log.e(TAG, "onCameraFrame: exception with smartCameraExtrinsicsCalibrator.recordRobotPose: "+e, e); }
        Se3_F64 estimated_camera_pose =
                this.smartCameraExtrinsicsCalibrator.detectedInImage("",imageFrameTime, pixelPosition,transformOfFeatureInVisualModel, frameTransform, matGray.width(), matGray.height());
        if(null != estimated_camera_pose) {
            Quaternion_F64 quaternionBoofCV = transform_to_quaternion_boofcv(estimated_camera_pose);
            detectedFeaturesClient.justPublishPose(
                    10000,
                    estimated_camera_pose.getX(), estimated_camera_pose.getY(), estimated_camera_pose.getZ(),
                    quaternionBoofCV.x, quaternionBoofCV.y, quaternionBoofCV.z, quaternionBoofCV.w);
        }
    }

    @NonNull
    private Quaternion_F64 transform_to_quaternion_boofcv(Se3_F64 transform_boofcv) {
        DenseMatrix64F rotation_mat_boofcv = transform_boofcv.getR();
        return rotation_mat_to_quaternion_boofcv(rotation_mat_boofcv);
    }

    @NonNull
    private Quaternion_F64 rotation_mat_to_quaternion_boofcv(DenseMatrix64F rotation_mat_boofcv) {
        Quaternion_F64 quaternionBoofCV = new Quaternion_F64();
        ConvertRotation3D_F64.matrixToQuaternion(rotation_mat_boofcv, quaternionBoofCV);
        return quaternionBoofCV;
    }

    DetectDescribePoint detDesc;
    ScoreAssociation scorer;
    AssociateDescription associate;
    AssociatePoints associatePoints;
    @NonNull
    private AssociatePoints setupAssociatePointsVariables(Class imageTypeGrayF32, Class imageRgbType) {
        if(null==detDesc) {
            detDesc = FactoryDetectDescribe.surfStable(new ConfigFastHessian(1, 2, 300, 1, 9, 4, 4), null, null, imageTypeGrayF32);
            scorer = FactoryAssociation.defaultScore(detDesc.getDescriptionType());
            associate = FactoryAssociation.greedy(scorer, Double.MAX_VALUE, true);
            associatePoints = new AssociatePoints(detDesc, associate, imageTypeGrayF32, imageRgbType);
        }
        return associatePoints;
    }

    @NonNull
    private Quaternion_F64 convertRosToBoofcvQuaternion(VisionTask visionTask) {
        return new Quaternion_F64(
                visionTask.getRelationToBase().getOrientation().getW(),
                visionTask.getRelationToBase().getOrientation().getX(),
                visionTask.getRelationToBase().getOrientation().getY(),
                visionTask.getRelationToBase().getOrientation().getZ());
    }

    @NonNull
    private List<DetectedTag> visionTaskFeaturesDetected(HashMap<RobotId, List<DetectedTag>> robotsDetected_, RobotId singleDummyRobotId_) {
        List<DetectedTag> robotFeatures__ = robotsDetected_.get(singleDummyRobotId_);
        if(null == robotFeatures__) {
            robotFeatures__ = new ArrayList<DetectedTag>();
            robotsDetected_.put(singleDummyRobotId_,robotFeatures__);
        }
        return robotFeatures__;
    }

    private enum FeatureModel {
        ROBOT_FEATURE, ROBOT_1_FEATURE, ROBOT_2_FEATURE, NON_ROBOT_FEATURE, FEATURE_WITHOUT_3D_LOCATION, INVALID_TAG_ID
    }

    private void renderGUI() {
        displayTargetAllocations_OpenCV();
    }

    private void imageProcessingNative(FocalLengthCalculator focalLengthCalc, CalcImageDimensions calcImgDim) {
        Log.i(TAG, "starting OpenCV processing ");
        String[] tags = new String[]{};
//        String[] tags = aprilTagsUmichOneShot(matGray.getNativeObjAddr(), matRgb.getNativeObjAddr(), tagDetectorPointer, tagSize_metres, focalLengthCalc.getFocal_length_in_pixels_x(), focalLengthCalc.getFocal_length_in_pixels_y(), calcImgDim.getPx_pixels(), calcImgDim.getPy_pixels());
//        System.out.println("\\n\\n\\n ---------- hardcoding to ZERO AprilTags detections ---------- \\n\\n\\n");
//        String[] tags = new String[]{};

        System.out.println("---------- detected " + tags.length + " tags ----------------------------------------------------------------------------------------------------");

        Time timeNow = DateAndTime.nowAsTime();
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
            if (null != detectedFeaturesClient) {
// ("tag ([0-9]+) at x=([0-9-]+\\.[0-9]+) y=([0-9-]+\\.[0-9]+) z=([0-9-]+\\.[0-9]+) roll=([0-9-]+\\.[0-9]+) pitch=([0-9-]+\\.[0-9]+) yaw=([0-9-]+\\.[0-9]+) qx=([0-9-]+\\.[0-9]+) qy=([0-9-]+\\.[0-9]+) qz=([0-9-]+\\.[0-9]+) qw=([0-9-]+\\.[0-9]+)");
                double x = Double.parseDouble(matcher.group(2));
                double y = Double.parseDouble(matcher.group(3));
                double z = Double.parseDouble(matcher.group(4));
//                double roll  = Double.parseDouble(matcher.group(5));
//                double pitch = Double.parseDouble(matcher.group(6));
//                double yaw   = Double.parseDouble(matcher.group(7));
                double qx = Double.parseDouble(matcher.group(5));
                double qy = Double.parseDouble(matcher.group(6));
                double qz = Double.parseDouble(matcher.group(7));
                double qw = Double.parseDouble(matcher.group(8));

                double xHom = Double.parseDouble(matcher.group(9));
                double yHom = Double.parseDouble(matcher.group(10));
                double zHom = Double.parseDouble(matcher.group(11));
                double qxHom = Double.parseDouble(matcher.group(12));
                double qyHom = Double.parseDouble(matcher.group(13));
                double qzHom = Double.parseDouble(matcher.group(14));
                double qwHom = Double.parseDouble(matcher.group(15));

                org.ros.rosjava_geometry.Vector3 translation_to_tag_in_robot_convention = new org.ros.rosjava_geometry.Vector3(x, y, z);
                //  NOTE !!
                // this corresponds to the fix in detect_feature_server.py line 278,
                // caused by Kaess' mixed coordinate system conventions in TagDetection.cc lines 142-145:
                // the quaternion is in camera-/optical-coordinate-system conventions ( rot = T.block ... rather than rot = MT.block ... ) ,
                // while the translation is in robot-coordinate-system convention (TagDetection.cc lines 133-141 :-  trans = MT.col(3).head(3)  )
                org.ros.rosjava_geometry.Quaternion quaternion_rotation_to_tag = new org.ros.rosjava_geometry.Quaternion(qz, -qx, -qy, qw);
                DetectedFeature feature = new DetectedFeature(APRIL_TAGS_KAESS_36_H_11, tagId, translation_to_tag_in_robot_convention, quaternion_rotation_to_tag);

                org.ros.rosjava_geometry.Vector3 translation_to_tag_in_robot_convention_hom = new org.ros.rosjava_geometry.Vector3(xHom, yHom, zHom);
                org.ros.rosjava_geometry.Quaternion quaternion_rotation_to_tag_hom = new org.ros.rosjava_geometry.Quaternion(qxHom, qyHom, qzHom, qwHom);
//                DetectedFeature feature_hom = new DetectedFeature(APRIL_TAGS_KAESS_36_H_11, Integer.toString(tagId_int), translation_to_tag_in_robot_convention_hom, quaternion_rotation_to_tag_hom);
                DetectedFeature feature_hom = new DetectedFeature(APRIL_TAGS_KAESS_36_H_11,
                        Integer.toString(tagId_int),
                        translation_to_tag_in_robot_convention,  // NOTE:  translation was good before; it's the rotation/orientation that was suspect
                        quaternion_rotation_to_tag_hom);

                org.ros.rosjava_geometry.Vector3 trans;
                org.ros.rosjava_geometry.Quaternion quat;
                DataTrack track;
                MedianFilter medianFilter;
                tryHomography();


                { // detection with translation, rotation from homography.c
                    // START Median filter over the translation - also effectively filters over the detected tag rotation: as this is the inverse of the camera-to-tag transform, the orientation of the tag relative to the camera is inverted and applied before the inverted-translation, so that the inverted-translation amplifies problems with the inverted-rotation
                    track = featureDataRecorderModeller.addDetection(feature_hom);
                    medianFilter = new MedianFilter(tag, feature_hom, track).invoke();
                    if (medianFilter.is()) {
                        continue; // not enough data yet
                    }
                    feature_hom = medianFilter.getFeature();
                    // END Median filter over the translation

                    detectedFeatures.add(feature_hom);


//                detectedFeaturesClient.reportDetectedFeature(tagId_int + 100, xHom, yHom, zHom, qxHom, qyHom, qzHom, qwHom);
                    trans = feature_hom.translation_to_tag_in_robot_convention;
                    quat = feature_hom.quaternion_rotation_to_tag;

//                detectedFeaturesClient.reportDetectedFeature(tagId_int, trans.getX(), trans.getY(), trans.getZ(), quat.getX(), quat.getY(), quat.getZ(), quat.getW());
                    // NOTE:  translation was good before; it's the rotation/orientation that was suspect
//                    detectedFeaturesClient.reportDetectedFeature(tagId_int, x, y, z, quat.getX(), quat.getY(), quat.getZ(), quat.getW());

                    // TODO - server-->camera send fixed pose 1) in registration 2) whenever the server wants to
                    if (!poseKnown) {
//                    localiseFromAFeatureClient.localiseFromAFeature(tagId_int + 100, xHom, yHom, zHom, qxHom, qyHom, qzHom, qwHom);
//                    localiseFromAFeatureClient.localiseFromAFeature(tagId_int, trans.getX(), trans.getY(), trans.getZ(), quat.getX(), quat.getY(), quat.getZ(), quat.getW());
                        // NOTE:  translation was good before; it's the rotation/orientation that was suspect
                        localiseFromAFeatureClient.localiseFromAFeature(tagId_int, x, y, z, quat.getX(), quat.getY(), quat.getZ(), quat.getW());
                    }
                }
//                      TODO - put markerPublisherNode function on the visualiser side
//                    //markerPublisherNode.publishMarker(String marker_namespace_, int marker_id_, String marker_text_, double x,double y,double z,double qx,double qy,double qz,double qw, String parent_frame_id, Time time_) {
//                    markerPublisherNode.publishMarker(MARKER_NAMESPACE, tagId_int, tagId, x, y, z, qx, qy, qz, qw, Naming.cameraFrameId(getCamNum()), timeNow);
//                    // TODO - use same variable for this and aa1_vos_android_catkin_ws___src/vos_aa1/src/vos_aa1/detect_feature_server.py
//                    // TODO - publish markers in detected_feature_server.py - once I have figured out what is going on with the RPY in that Python code
//                    // TODO -   ... or use C++ as detected_feature_server.cpp
//                }
            } else {
                eventLog_DetectedFeaturesClient_notAvailable(tagId);
            }
        }
    }

    private void dealWithDetectedFeatures(HashMap<RobotId, List<DetectedTag>> detectedFeatures) {
        for(RobotId robotId_ : detectedFeatures.keySet()) {
            HashMap<RobotId, List<DetectedTag>> detectedFeaturesSubset = new HashMap<RobotId, List<DetectedTag>>();
            detectedFeaturesSubset.put(robotId_, detectedFeatures.get(robotId_));
            if (robotId_.idString().startsWith(VisionTaskType.LOCALISE_CAMERA.name())) {
                doNothing_localiseCameraFromFeatures(detectedFeaturesSubset);
            } else if (robotId_.idString().startsWith(VisionTaskType.LOCALISE_EXTERNAL.name())) {
                calcAndReportRobotPose(detectedFeaturesSubset);
            } else {

            }
        }
    }

    private void doNothing_localiseCameraFromFeatures(HashMap<RobotId, List<DetectedTag>> robots) {

    }

    /**
     * Filter and smooth the robot pose estimates for one time period
     * - no motion model -
     * from the poses of the detected features of that robot
     * and send that pose estimate to the VOS Server.
     *
     * @param robots map of robot ids to detected features.
     */
    private void calcAndReportRobotPose(HashMap<RobotId, List<DetectedTag>> robots) {
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
                Log.i(robotId_.idString(), "calcAndReportRobotPose: Tag id = "+detectedTag.getTag_id()+" euler angles are "+eulerAngles[0]+" , "+eulerAngles[1]+" , "+eulerAngles[2]+"");
                double zRotation = eulerAngles[0];
                if(zRotation>Math.PI) {zRotation = zRotation - (Math.PI*2.0); } //e.g. convert 345 degrees to -15 degrees
                Log.i(robotId_.idString(), "calcAndReportRobotPose: Tag id = "+detectedTag.getTag_id()+" zRotation is "+zRotation);
                zRotationSum += zRotation;
                if(zRotation<0.0) {zRotationPositive=(Math.PI*2.0)+zRotation;} else {zRotationPositive=zRotation;}
                Log.i(robotId_.idString(), "calcAndReportRobotPose: Tag id = "+detectedTag.getTag_id()+" zRotationPositive is "+zRotationPositive);
                if(1 < numTagsForRobot) {
                    Log.w(robotId_.idString(), "calcAndReportRobotPose: calcAndReportRobotPose: UNWEIGHTED MEAN zRotationMean ONLY WORKS FOR 2 TAGS");
                    double diff = Math.abs(zRotationPositivePrev - zRotationPositive);
                    double diffDiv2 = diff/2.0;
                    Log.i(robotId_.idString(), "calcAndReportRobotPose: Tag id = "+detectedTag.getTag_id()+" diffDiv2 = "+diffDiv2);
                    if(diff > Math.PI) {
                        diffDiv2 = (Math.PI-diffDiv2);
                        Log.i(robotId_.idString(), "calcAndReportRobotPose: Tag id = "+detectedTag.getTag_id()+" diffDiv2 = "+diffDiv2);
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
                    Log.i(robotId_.idString(), "calcAndReportRobotPose: zRotationIncrementalMean is "+zRotationIncrementalMean);
                meanSensorToTargetTransformRot = ConvertRotation3D_F64.eulerToMatrix(EulerType.ZYX, zRotationIncrementalMean, 0.0, 0.0, null);
                    Log.i(robotId_.idString(), "calcAndReportRobotPose: meanSensorToTargetTransformRot = "+meanSensorToTargetTransformRot);
                Se3_F64 meanSensorToTargetViaTransform = new Se3_F64();
                meanSensorToTargetViaTransform.setTranslation(xTranslationMean,yTranslationMean,zTranslationMean);
                meanSensorToTargetViaTransform.setRotation(meanSensorToTargetTransformRot);
                Quaternion_F64 meanSensorToTargetViaTransformQuat = new Quaternion_F64();
                ConvertRotation3D_F64.matrixToQuaternion(meanSensorToTargetTransformRot, meanSensorToTargetViaTransformQuat);
                    Log.i(robotId_.idString(), "calcAndReportRobotPose: estimated pose from "+numTagsForRobot+" tag detections");
                    Log.i(robotId_.idString(), "calcAndReportRobotPose: detectedFeaturesClient.reportDetectedFeature");
                detectedFeaturesClient.reportDetectedFeature(
                    robotId_.idString(), "calcAndReportRobotId: REQUEST_ID UNKNOWN",
                        /** visionTask.getRobotId(), visionTask.getRequestId(), */
                    90000+robotId_.idInt(), //90000+1, 90000+557, 90000+999999, etc: allow for publishing more than one frame per tag, with the tag reported as [frame type]+[tag id]
                    xTranslationMean, yTranslationMean, zTranslationMean,
                    meanSensorToTargetViaTransformQuat.x,meanSensorToTargetViaTransformQuat.y,meanSensorToTargetViaTransformQuat.z,meanSensorToTargetViaTransformQuat.w);
            }
        }
        /* end Dev: part of robot visual model */
    }



    private void drawMarkeLocationOnDisplay_BoofCV(FiducialDetector<GrayF32> detector, int detectionOrder_, VisionTask visionTask) {
        // would draw the task number and so on if I had a text output function
        if(null == visionTask.getRobotId()) {
            drawMarkerLocationOnDisplay_BoofCV(detector, detectionOrder_, FeatureModel.ROBOT_FEATURE);
        } else if(visionTask.getRobotId().contains("1")) {
            drawMarkerLocationOnDisplay_BoofCV(detector, detectionOrder_, FeatureModel.ROBOT_1_FEATURE);
        } else if(visionTask.getRobotId().contains("2")) {
            drawMarkerLocationOnDisplay_BoofCV(detector, detectionOrder_, FeatureModel.ROBOT_2_FEATURE);
        } else {
            drawMarkerLocationOnDisplay_BoofCV(detector, detectionOrder_, FeatureModel.ROBOT_FEATURE);
        }
    }

    private void drawMarkeLocationOnDisplay_BoofCV_validTagNotInTask(FiducialDetector<GrayF32> detector, int detectionOrder_) {
        drawMarkerLocationOnDisplay_BoofCV(detector, detectionOrder_, FeatureModel.ROBOT_FEATURE);
    }

    private void drawMarkeLocationOnDisplay_BoofCV_no3dData(FiducialDetector<GrayF32> detector, int detectionOrder_) {
        drawMarkerLocationOnDisplay_BoofCV(detector, detectionOrder_, FeatureModel.FEATURE_WITHOUT_3D_LOCATION);
    }

    private void drawMarkeLocationOnDisplay_BoofCV_invalidTagId(FiducialDetector<GrayF32> detector, int detectionOrder_) {
        drawMarkerLocationOnDisplay_BoofCV(detector, detectionOrder_, FeatureModel.INVALID_TAG_ID);
    }

    private void drawMarkerLocationOnDisplay_BoofCV(FiducialDetector<GrayF32> detector, int index_of_tag_, FeatureModel fm ) {
        Log.i("drawMarkerOnDisplay_BCV","tag_id = "+index_of_tag_);
        Point2D_F64 locationPixel = new Point2D_F64();
        detector.getImageLocation(index_of_tag_, locationPixel);        // pixel location in input image
        System.out.println("2D Image Location = "+locationPixel);
        Log.i("drawMarkerOnDisplay_BCV","2D Image Location = "+locationPixel);
        displayTagCentre_OpenCV((int)locationPixel.x, (int)locationPixel.y, fm);
//                        VisualizeFiducial.drawLabel(locationPixel, "" + detector.getId(tag_id), g2);
    }

    private void variousUnusedAttemptsAtCoordinateSystemCorrection() {
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
    }

    private void tryHomography() {
        ////  2017_05_11 homography attempt - comment out the non-homography
//                // START Median filter over the translation - also effectively filters over the detected tag rotation: as this is the inverse of the camera-to-tag transform, the orientation of the tag relative to the camera is inverted and applied before the inverted-translation, so that the inverted-translation amplifies problems with the inverted-rotation
//                DataTrack track = featureDataRecorderModeller.addDetection(feature);
//                MedianFilter medianFilter = new MedianFilter(tag, feature, track).invoke();
//                if (medianFilter.is()) {
//                    continue; // not enough data yet
//                }
//                feature = medianFilter.getFeature();
//                // END Median filter over the translation
//
//                detectedFeatures.add(feature);
//
////                detectedFeaturesClient.reportDetectedFeature(tagId_int, x, y, z, qx, qy, qz, qw);
//                org.ros.rosjava_geometry.Vector3 trans = feature.translation_to_tag_in_robot_convention;
//                org.ros.rosjava_geometry.Quaternion quat = feature.quaternion_rotation_to_tag;
//                detectedFeaturesClient.reportDetectedFeature(tagId_int, trans.getX(), trans.getY(), trans.getZ(), quat.getX(), quat.getY(), quat.getZ(), quat.getW());
//
//                if (!poseKnown) {
////                    localiseFromAFeatureClient.localiseFromAFeature(tagId_int, x, y, z, qx, qy, qz, qw);
//                    localiseFromAFeatureClient.localiseFromAFeature(tagId_int, trans.getX(), trans.getY(), trans.getZ(), quat.getX(), quat.getY(), quat.getZ(), quat.getW());
//                }
////  end 2017_05_11 homography attempt - comment out the non-homography
    }


    private void rotateImage() {
        //        Core.flip(matGray,matGray,1);
//        Core.flip(matRgb,matRgb,1);
    }

    private void exitIfNotImageProcessing() {
        //      should be able to use  disableView()
//        if(!runImageProcessing){
//            return matRgb;
//        }
    }

    private void focalLengthCorrections() {
        // TODO - needs API 21
//        float[] f = characteristics.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS);
//        for (float d : f) {
//            Logger.logGeneral("LENS_INFO_AVAILABLE_FOCAL_LENGTHS : " + d);
//        }
    }

    private void rotationCorrection() {
        //        Display display = ((WindowManager) this.getSystemService(Context.WINDOW_SERVICE)).getDefaultDisplay();
//        int rotation = display.getRotation();
//        Log.d(TAG, "rotation = " + rotation);
    }

    private void registerAsVisionSource(String logTag) {                //// TODO - move into control loop
        if (!registeredAsVisionSource & null != registerVisionSourceClient) {
            Log.i(logTag,"start registering as a vision source");       //// TODO - timing here  c[camera_num]-f[frameprocessed]
            Log.i(TAG,"onCameraFrame: registering as a vision source.");
            registerVisionSourceClient.registerVisionSource();
            registeredAsVisionSource = true;
            Log.i(logTag,"finished registering as a vision source");    //// TODO - timing here  c[camera_num]-f[frameprocessed]
        }
    }

    private void updateLocationFromDetectedFeature(int tag_id, String logTagTag, Se3_F64 sensorToTargetViaTransform, Quaternion_F64 sensorToTargetViaTransformQuat) {
        if (!poseKnown) {
            localiseFromAFeatureClient.localiseFromAFeature(tag_id,
                    sensorToTargetViaTransform.getX(), sensorToTargetViaTransform.getY(), sensorToTargetViaTransform.getZ(),
                    sensorToTargetViaTransformQuat.x,sensorToTargetViaTransformQuat.y,sensorToTargetViaTransformQuat.z,sensorToTargetViaTransformQuat.w);
            //// TODO - timing here  c[camera_num]-f[frameprocessed]-i[iteration]-t[tagid]
            Log.i(logTagTag,"after localiseFromAFeatureClient.localiseFromAFeature");
        }
    }

    private void updateTrackingData(List<DetectedTag> robotFeatures) {
        double averageX = 0.0, averageY = 0.0;
        int numRobotFeaturesDetected=0;
        for (DetectedTag robotFeature : robotFeatures) {
            numRobotFeaturesDetected++;
            averageX = averageX+robotFeature.getSensorToTargetViaTransform().getX();
            averageY = averageY+robotFeature.getSensorToTargetViaTransform().getY();
        }
        averageX = averageX/numRobotFeaturesDetected;
        averageY = averageY/numRobotFeaturesDetected;
        Point2D_F64 averageLocation = new Point2D_F64(averageX,averageY);
        synchronized (robotFeatureTrackingMonitor) {
            robotFeatureTrackingAverages.add(averageLocation);
            robotFeatureTracking.add(robotFeatures);
        }
    }

    private HashMap<Integer, double[]> fixedLandmarks;


    private Se3_F64 updatePoseEstimate(List<DetectedTag> landmarkFeatures) {
        loadLandmarkFeaturesOnce();

        double[] worldX = new double[landmarkFeatures.size()];
        double[] worldY = new double[landmarkFeatures.size()];
        double[] worldZ = new double[landmarkFeatures.size()];
        double[] pixelsX = new double[landmarkFeatures.size()];
        double[] pixelsY = new double[landmarkFeatures.size()];
        int i_ = 0;
        for (DetectedTag tag:
             landmarkFeatures) {
            double[] worldCoordinates = fixedLandmarks.get(tag.getTag_id());
            if(null != worldCoordinates) {
                Point2D_F64 pixelLocation = tag.getLocationPixel();
                worldX[i_] = worldCoordinates[0];
                worldY[i_] = worldCoordinates[1];
                worldZ[i_] = worldCoordinates[2];
                pixelsX[i_] = pixelLocation.getX();
                pixelsY[i_] = pixelLocation.getY();
                i_++;
            }
        }
        PoseFrom3D2DPointMatches estimator = new LocalisePnP_BoofCV();
        FocalLengthCalculator focalLengthCalculator = new FocalLengthCalculator();
        CalcImageDimensions calcImgDim = new CalcImageDimensions();
        //CameraPinhole(double fx, double fy, double skew, double cx, double cy, int width, int height)
        CameraPinholeRadial cameraIntrinsics = new CameraPinholeRadial(
                FocalLengthCalculator.getFocal_length_in_pixels_x(matGray.width()), FocalLengthCalculator.getFocal_length_in_pixels_y(matGray.height()),
                0, calcImgDim.getPx_pixels(), calcImgDim.getPy_pixels(), (int)matGray.size().width, (int)matGray.size().height);

        return estimator.estimateCameraPoseFrom3D2DPointMatches(
                cameraIntrinsics,//CameraIntrinsics.exampleCameraPinholeRadial(),  /*  TODO - HARDCODING in here */
                landmarkFeatures.size(), worldX, worldY, worldZ, pixelsX, pixelsY);

    }

    private void loadLandmarkFeaturesOnce() {
        /*  TODO - HARDCODING  */
        if(null == fixedLandmarks) {fixedLandmarks = landmarkFeatureLoader.loadLandmarkFeatures();}
        /*  TODO - HARDCODING  */
    }

    private void eventLog_DetectedFeaturesClient_notAvailable(String tagId) {
        System.out.print("MainActivity: onCameraFrame: detectedFeaturesClient is null: cannot report the poses of detected tags");
        System.out.println(tagId);
        System.out.println("-------------------------------------------------------");
    }

    private void blankScreenOutput_OpenCV(String logTag) {
        System.out.println("onCameraFrame: screenLocked = true at frame "+framesProcessed+": setting output matrices to black");
        Scalar blackScalar = new Scalar(0); //,CvType.CV_8UC4
        matRgb.setTo(blackScalar);
//// TODO - timing here  c[camera_num]-f[frameprocessed]-i[iteration]-t[tagid]
        Log.i(logTag,"after matRgb.setTo(blackScalar);");
    }

    private void displayTargetAllocations_OpenCV() {
        if(allocatedTargets.get(ROBOT_ALLOCATION_KEY)) {
            double[] red = new double[] {255d,0d,0d,255d};
            for(int pixel_u = 10; pixel_u<30; pixel_u++) {
                for(int pixel_v = 10; pixel_v<30; pixel_v++) {
                    matRgb.put(pixel_u, pixel_v, red);
                }
            }
        }
        if(allocatedTargets.get(TARGET_ALLOCATION_KEY)) {
            double[] blue = new double[] {0d,0d,255d,255d};
            for(int pixel_u = 30; pixel_u<50; pixel_u++) {
                for(int pixel_v = 10; pixel_v<30; pixel_v++) {
                    matRgb.put(pixel_u, pixel_v,  blue);
                }
            }
        }
    }

    private void displaySurf_OpenCV(int tagCentreX, int tagCentreY) {
        Log.i("displaySurf_OpenCV","start");
        Log.i("displaySurf_OpenCV","tag centre at "+tagCentreX+", "+tagCentreY);
        int xmin = tagCentreX-2; if(xmin < 0) {xmin = 0;}
        int xmax = tagCentreX+2; if(xmax > matRgb.width()) {xmax = matRgb.width();}
        int ymin = tagCentreY-2; if(ymin < 0) {ymin = 0;}
        int ymax = tagCentreY+2; if(ymax > matRgb.height()) {ymax = matRgb.height();}
        double[] colour;
        colour = new double[] {0d,255d,0d,255d};
        Log.i("displaySurf_OpenCV","SURF centre displayed at "+xmin+"_"+xmax+", "+ymin+"_"+ymax);
        for(int pixel_u = xmin; pixel_u<xmax; pixel_u++) {
            for(int pixel_v = ymin; pixel_v<ymax; pixel_v++) {
                matRgb.put(pixel_v, pixel_u, colour);   // int put(int row, int col, double... data)
            }
        }
        Log.i("displayTagCentre_OpenCV","end");
    }

    private void displayTagCentre_OpenCV(int tagCentreX, int tagCentreY, FeatureModel fm) {
        Log.i("displayTagCentre_OpenCV","start");
        int squareSize=10;
        double[] colour;
        switch (fm) {
            case ROBOT_FEATURE:
                colour = new double[] {0d,255d,0d,255d};
                drawSquare(tagCentreX, tagCentreY, colour, squareSize);
                break;
            case ROBOT_1_FEATURE:
                colour = new double[] {0d,255d,0d,255d};
                drawSquare(tagCentreX, tagCentreY, colour, squareSize+4);
                colour = new double[] {0d,200d,200d,255d};
                drawSquare(tagCentreX+4, tagCentreY+4, colour, squareSize-2); // diagonal quartering - top-right to bottom-left ~ forward slash
                drawSquare(tagCentreX-4, tagCentreY-4, colour, squareSize-2);
                break;
            case ROBOT_2_FEATURE:
                colour = new double[] {0d,255d,0d,255d};
                drawSquare(tagCentreX, tagCentreY, colour, squareSize+4);
                colour = new double[] {150d,150d,150d,255d};
                drawSquare(tagCentreX-4, tagCentreY+4, colour, squareSize-2); // diagonal quartering - top-left to bottom-right ~ back slash
                drawSquare(tagCentreX+4, tagCentreY-4, colour, squareSize-2);
                break;
            case NON_ROBOT_FEATURE:
                colour = new double[] {125d,125d,255d,255d};
                drawSquare(tagCentreX, tagCentreY, colour, squareSize);
                break;
            case FEATURE_WITHOUT_3D_LOCATION:
                colour = new double[] {220d,75d,0d,255d};
                drawSquare(tagCentreX, tagCentreY, colour, squareSize);
                break;
            case INVALID_TAG_ID:
                colour = new double[] {255d,0d,0d,255d};
                drawSquare(tagCentreX, tagCentreY, colour, squareSize);
                break;
            default:
                colour = new double[] {125d,125d,125d,255d};
                drawSquare(tagCentreX, tagCentreY, colour, squareSize);
        }
        Log.i("displayTagCentre_OpenCV","tag centre at "+tagCentreX+", "+tagCentreY);
        Log.i("displayTagCentre_OpenCV","end");
    }

    private void drawSquare(int tagCentreX, int tagCentreY, double[] colour, int squareSize) {
        int squareHalfSize=squareSize/2;
        int xmin = tagCentreX-squareHalfSize;
        if(xmin < 0) {xmin = 0;}
        int xmax = tagCentreX+squareHalfSize;
        if(xmax > matRgb.width()) {xmax = matRgb.width();}
        int ymin = tagCentreY-squareHalfSize;
        if(ymin < 0) {ymin = 0;}
        int ymax = tagCentreY+squareHalfSize;
        if(ymax > matRgb.height()) {ymax = matRgb.height();}
        Log.i("displayTagCentre_OpenCV","tag centre displayed at "+xmin+"_"+xmax+", "+ymin+"_"+ymax);
        for(int pixel_u = xmin; pixel_u<xmax; pixel_u++) {
            for(int pixel_v = ymin; pixel_v<ymax; pixel_v++) {
                matRgb.put(pixel_v, pixel_u, colour);   // int put(int row, int col, double... data)
            }
        }
    }


    /* Dev: part of robot visual model */
    private boolean isPartOfRobotVisualModel(int tag_id) {
        return Hardcoding.isPartOfRobotVisualModel(tag_id);
    }

    /* Dev: part of knowledge about landmarks */
    private boolean isALandmark(int tag_id) {
        return Hardcoding.isALandmark(tag_id);
    }

    private boolean isAnOutlier(DetectedFeature feature_) {
        return false;
    }

    private void calculateFocalLength_a(Camera camera) {
        double focal_length_in_pixels_x;
        double focal_length_in_pixels_y;
        if(null!=camera) {
            float focalLengthInMetres = camera.getParameters().getFocalLength();
            System.out.println("MainActivity: onCameraFrame: focalLengthInMetres="+focalLengthInMetres);
            float horizontalAngleView = camera.getParameters().getHorizontalViewAngle();
            System.out.println("MainActivity: onCameraFrame: horizontalAngleView="+horizontalAngleView);
            double focal_length_in_pixels = (camera.getParameters().getPictureSize().width * 0.5) / tan(horizontalAngleView * 0.5 * PI/180.0);
            System.out.println("MainActivity: onCameraFrame: focal_length_in_pixels="+focal_length_in_pixels);
        } else {
            System.out.println("MainActivity: onCameraFrame: AndroidCameraAdapterForDepricatedApi.getCameraInstance() returns null");
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
                System.out.println("MainActivity: onCameraFrame: focalLengthInMetres="+focalLengthInMetres);
                float horizontalAngleView = camera.getParameters().getHorizontalViewAngle();
                System.out.println("MainActivity: onCameraFrame: horizontalAngleView="+horizontalAngleView);
                focal_length_in_pixels_x = (camera.getParameters().getPictureSize().width * 0.5) / tan(horizontalAngleView * 0.5 * PI/180.0);
                System.out.println("MainActivity: onCameraFrame: focalLengthCalc.getFocal_length_in_pixels_x()="+focal_length_in_pixels_x);
                float verticalAngleView = camera.getParameters().getVerticalViewAngle();
                System.out.println("MainActivity: onCameraFrame: getVerticalViewAngle()="+verticalAngleView);
                focal_length_in_pixels_y = (camera.getParameters().getPictureSize().width * 0.5) / tan(verticalAngleView* 0.5 * PI/180.0);
                System.out.println("MainActivity: onCameraFrame: focalLengthCalc.getFocal_length_in_pixels_y()="+focal_length_in_pixels_y);
            } else {
                System.out.println("MainActivity: onCameraFrame: could not get a camera at all : using the Camera 1 API");
            }
        }
    }

    /** Get the focal length in pixels for AprilTags --> position calculation. */
    private void calculateFocalLength_b() {  /* See  https://play.google.com/books/reader?id=hb8FCgAAQBAJ&printsec=frontcover&output=reader&hl=en_GB&pg=GBS.PA103.w.12.0.0  pp124-126  */
        float[] estimatedFocusedDistances = {9000.0f,9000.0f,9000.0f};      // dummy values, overridden in cameraParameters().getFocusDistances(estimatedFocusedDistances)
        cameraParameters().getFocusDistances(estimatedFocusedDistances);    // focus distances in meters. the distances from the camera to where an object appears to be in focus. The object is sharpest at the optimal focus distance. The depth of field is the far focus distance minus near focus distance.param 'output' must be a float array with three elements. Near focus distance, optimal focus distance, and far focus distance will be filled in the array
        System.out.println("MainActivity: onCameraFrame: estimatedFocusedDistances = "+ Arrays.toString(estimatedFocusedDistances));
        MatOfDouble mProjectionCV = new MatOfDouble();
        mProjectionCV.create(3,3, CvType.CV_64FC1);
        final double fovAspectRatio = fieldOfViewX() / fieldOfViewY();
        double diagonalPixels = Math.sqrt( (Math.pow(matGray.size().width, 2.0)) + (Math.pow(matGray.size().width/fovAspectRatio, 2.0)) );
        double diagonalFov = Math.sqrt( (Math.pow(fieldOfViewX(), 2.0)) + (Math.pow(fieldOfViewY(), 2.0)) );
        double focalLengthPixels = diagonalPixels / (2.0 * Math.tan(0.5 * diagonalFov * Math.PI/180.0f ));
        double focal_length_in_pixels_x = ( matGray.size().width / (2.0 * Math.tan(0.5 * fieldOfViewX() * Math.PI/180.0f)) );
        double focal_length_in_pixels_y = ( matGray.size().height / (2.0 * Math.tan(0.5 * fieldOfViewY() * Math.PI/180.0f)) );
        System.out.println("MainActivity: onCameraFrame: "
                +"  OpenCV matrix: width pixels="+matGray.size().width+", height pixels="+matGray.size().height
                +", preview: width pixels="+cameraParameters().getPreviewSize().width+", height pixels="+cameraParameters().getPreviewSize().height
                +", zoom value="+cameraParameters().getZoom()+", zoom as percentage="+cameraParameters().getZoomRatios().get(cameraParameters().getZoom()).intValue()
                +", fieldOfView: X rads="+fieldOfViewX()+", Y rads="+fieldOfViewY()
                +", fovAspectRatio="+fovAspectRatio+", diagonalPixels="+diagonalPixels
                +", diagonalFov rads="+diagonalFov+", focalLengthPixels - diagonal="+focalLengthPixels);
        System.out.println("MainActivity: onCameraFrame: "
                +", focal length x = "+  focal_length_in_pixels_x
                +", focal length y = "+  focal_length_in_pixels_y );
    }

    /*** implement DimmableScreen *****************************************************************/

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

    @Override
    public void screenOn(float percentBrightness) {
        screenLocked = false;
//        2017_08_24 - allow preferred/adaptive brightness
//        if(0.0 > percentBrightness) {
//            percentBrightness = 0.0f;
//        } else
        if (1.0 < percentBrightness) {
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

    @Override
    public void displayGrey() { displayRgb = false; }

    @Override
    public void displayRgb() { displayRgb = true; }

    /*** end implement DimmableScreen *****************************************************************/


    @Override
    public void setPose(double[] poseXyz, double[] orientationQuaternionXyzw_) {
        System.out.println("setPose( pose xyz="+Arrays.toString(poseXyz)+", quat xyzw="+Arrays.toString(orientationQuaternionXyzw_)+")");
        this.position = poseXyz;
        this.orientation = orientationQuaternionXyzw_;
        this.poseKnown = true;
    }

    public void setPose(Pose pose_) {
        setPose(
                new double[]{pose_.getPosition().getX(),pose_.getPosition().getY(),pose_.getPosition().getZ()},
                new double[]{pose_.getOrientation().getX(),pose_.getOrientation().getY(),pose_.getOrientation().getZ(),pose_.getOrientation().getW()});
    }


    public void resetToPoseUnknown() {
        Log.i(TAG,"resetToPoseUnknown(): resetting to null pose, and poseKnown   = false");
        this.position    = NULL_POSITION;
        this.orientation = NULL_ORIENTATION;
        this.poseKnown   = false;
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
//    public native long newTagDetectorKaess();   // Apriltags          //  // 2017_08_23
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

    @Override
    public void startObstacleDetection() {
        checkPermissions();

        determiningFreeFloorspace = "HSV && Texture";
        if(!runImageProcessing) {
            _cameraBridgeViewBase.enableView();
        }
    }

    @Override
    public void startObstacleDetectionHSV() {
        checkPermissions();

        determiningFreeFloorspace = "HSV";
        if(!runImageProcessing) {
            _cameraBridgeViewBase.enableView();
        }
    }

    @Override
    public void startObstacleDetectionTexture() {
        checkPermissions();

        determiningFreeFloorspace = "Texture";
        if(!runImageProcessing) {
            _cameraBridgeViewBase.enableView();
        }
    }

    public void startObstacleDetectionAndProject() {
        checkPermissions();

        determiningFreeFloorspace = "HSV && Texture && display_projected";
        if(!runImageProcessing) {
            _cameraBridgeViewBase.enableView();
        }
    }

    @Override
    public void startObstacleDetectionAndProjectHistory() {
        checkPermissions();

        determiningFreeFloorspace = "HSV && Texture && display_projected && display_hist_projected";
        if(!runImageProcessing) {
            _cameraBridgeViewBase.enableView();
        }
    }

    @Override
    public void stopObstacleDetection() {

        checkPermissions();
        determiningFreeFloorspace = null;
        if(!runImageProcessing) {
            _cameraBridgeViewBase.disableView();
        }
    }



    private void checkPermissions() {
        PermissionsChecker permissionsChecker = new PermissionsChecker(this);
        permissionsChecker.havePermissionAndRequest(Manifest.permission.ACCESS_WIFI_STATE, "This application requires access to wifi: please enable via Settings -> Apps (or similar) -> Your app -> Permissions");
        permissionsChecker.havePermissionAndRequest(Manifest.permission.CHANGE_WIFI_STATE, "This application requires access to wifi: please enable via Settings -> Apps (or similar) -> Your app -> Permissions");
        permissionsChecker.havePermissionAndRequest(Manifest.permission.INTERNET, "This application requires access to the internet: please enable via Settings -> Apps (or similar) -> Your app -> Permissions");
        permissionsChecker.havePermissionAndRequest(Manifest.permission.WAKE_LOCK, "This application requires access to wake the device: please enable via Settings -> Apps (or similar) -> Your app -> Permissions");
        permissionsChecker.havePermissionAndRequest(Manifest.permission.READ_EXTERNAL_STORAGE, "This application requires access to read stored data: please enable via Settings -> Apps (or similar) -> Your app -> Permissions");
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
    public void resetExtrinsicCalibration(){
        Log.i(TAG,"resetExtrinsicCalibration(): recording observations, then reseting the pose and observations, and starting a new recording directory.");
        this.smartCameraExtrinsicsCalibrator.logObservations2ToFile();  // check that the observation data is recorded ...
        this.smartCameraExtrinsicsCalibrator.uncalibrated();            // ... observations dropped ...
        this.resetToPoseUnknown();                                      // ... pose is now unknown ...
        this.resetDataOutputDirectory();                                // ... start recording to a new dir
    }

    private enum DetectionType {
        ONE_IMAGE, CONTINUOUS, STOP;
    }

    private DetectionType DETECTION_TYPE = DetectionType.CONTINUOUS;

    public void extrinsicsCalibration(ManagementCommand command) {
        Log.i(TAG,"extrinsicsCalibration: start");
        switch (command) {
            case EXTERNAL_CALIBRATION_CAPTURE_ONE_IMAGE:
                Log.i(TAG,"extrinsicsCalibration: setting up to capture one image and pose.");
                DETECTION_TYPE = DetectionType.ONE_IMAGE;
                break;
            case EXTERNAL_CALIBRATION_CAPTURE_STOP:
                Log.i(TAG,"extrinsicsCalibration: setting up to stop capture.");
                DETECTION_TYPE = DetectionType.STOP;
                break;
            case EXTERNAL_CALIBRATION_CAPTURE_CONTINUOUS:
                Log.i(TAG,"extrinsicsCalibration: setting up to capture continuously.");
                DETECTION_TYPE = DetectionType.CONTINUOUS;
                break;
            case EXTERNAL_CALIBRATION_LIST_OBSERVATIONS:
                Log.i(TAG,"extrinsicsCalibration: listing observations.");
                smartCameraExtrinsicsCalibrator.listObservations();
                break;
            case EXTERNAL_CALIBRATION_REQUEST_CURRENT_ROBOT_POSE:
                java.util.Date now = nowAsDate();
                Log.i(TAG,"extrinsicsCalibration: setting up to current robot pose at "+now+" = "+now.getTime()+"ms");
                FrameTransform frameTransform = smartCameraExtrinsicsCalibrator.askRobotForItsPoseFrame();
                now = nowAsDate();
                Log.i(TAG,"extrinsicsCalibration: received current robot pose at "+now+" = "+now.getTime()+"ms");
                Log.i(TAG,"extrinsicsCalibration: received current robot pose = "+frameTransform+" at "+now+" = "+now.getTime()+"ms");
                break;
            case EXTERNAL_CALIBRATION_CALIBRATE_AND_USE:
                smartCameraExtrinsicsCalibrator.estimateExtrinsics();
                break;
            case RECORD_DATA_ONCE:
                RECORD_DATA = true;
                RECORD_DATA_RECORDED = false;
                Log.i(TAG, "extrinsicsCalibration: "+command.name()+": " + logMessage_recordDataStatus());
                break;
            case RECORD_DATA_CONTINUOUS:
                RECORD_DATA = true;
                RECORD_DATA_CONTINUOUS = true;
                RECORD_DATA_RECORDED = false;
                Log.i(TAG, "extrinsicsCalibration: "+command.name()+": " + logMessage_recordDataStatus());
                break;
            case RECORD_DATA_STOP:
                RECORD_DATA = false;
                RECORD_DATA_CONTINUOUS = false;
                RECORD_DATA_RECORDED = true;
                Log.i(TAG, "extrinsicsCalibration: "+command.name()+": " + logMessage_recordDataStatus());
                break;
            case RECORD_NEXT_FRAME:
                RECORD_FRAMES = 1;
                break;
            case RECORD_RESET_OUTPUT_DIRECTORY:
                Log.i(TAG,"RECORD_RESET_OUTPUT_DIRECTORY --> resetDataOutputDirectory()");
                resetDataOutputDirectory();
                break;
            case TEST_MATLAB_POSE_EST_kitchen_realPixels:
                atestMatlabPoseEstimation_kitchen_realPixels();
                break;
            case TEST_MATLAB_POSE_EST_kitchen_projectedPixels:
                atestMatlabPoseEstimation_kitchen_projectedPixels();
                break;
            case TEST_MATLAB_POSE_EST_square_projectedPixels:
                atestMatlabPoseEstimation_square_projectedPixels();
                break;
            case CHECK_TIME_SYNCH_AND_TF_AVAILABLE:
                CHECK_TIME_SYNCH = 50;
                break;
            case EXPIRE_TASKS_ON_DETECTION_COUNT:
                vosTaskSet.expireTasksOnDetection(true);
                break;
            case EXPIRE_TASKS_ON_EXECUTION_COUNT:
                vosTaskSet.expireTasksOnDetection(false);
                break;
            case FIDUCIAL_MASK_WITH_HSV_ON:
                manage_maskFiducialWithHSV(true);
                break;
            case FIDUCIAL_MASK_WITH_HSV_OFF:
                manage_maskFiducialWithHSV(false);
                break;
            default:
                Log.e(TAG,"extrinsicsCalibration: ManagementCommand not recognised: '"+command+"': throwing a RuntimeException.");
                throw new RuntimeException("extrinsicsCalibration: ManagementCommand '"+command+"' not recognised");
        }
    }


    private void atestMatlabPoseEstimation_kitchen_realPixels() {
        vosTaskAssignmentSubscriberNode.atestMatlabPoseEstimation_kitchen_realPixels();
    }

    private void atestMatlabPoseEstimation_kitchen_projectedPixels() {
        Log.w(TAG,"atestMatlabPoseEstimation_kitchen_projectedPixels: does nothing yet.");
    }

    private void atestMatlabPoseEstimation_square_projectedPixels() {
        Log.w(TAG,"atestMatlabPoseEstimation_square_projectedPixels: does nothing yet.");
    }


    @NonNull
    private String logMessage_recordDataStatus() {
        return "RECORD_DATA = " + RECORD_DATA
                + ", "+"RECORD_DATA_CONTINUOUS = " + RECORD_DATA_CONTINUOUS
                + ", "+"RECORD_DATA_RECORDED = " + RECORD_DATA_RECORDED;
    }

    @Override
    public void relocalise() {
            System.out.println("MainActivity: relocalise()");
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
        convertPreviewForBoofCV(last_frame_bytes,_cameraBridgeViewBase.camera());  // updates the 'image' variable
    }

    /**
     * Log some data: TODO - implement as a generic data publisher.
     */
    void logData(String dataLabel,String dataValue) {
        Log.i(TAG,"logData: logging data: label="+dataLabel+", value='"+dataValue+"'");
    }


/* start - copy from boofcv.android.gui.VideoRenderProcessing */ // TODO - use it properly
    // Type of BoofCV image
    ImageType<boofcv.struct.image.GrayF32> imageTypeGray = new ImageType<GrayF32>(GRAY,F32,1);
    boofcv.struct.image.Planar<GrayF32> imageRgb         = new boofcv.struct.image.Planar<GrayF32>(GrayF32.class,1,1,3);
    boofcv.struct.image.Planar<GrayF32> imageHsv         = new boofcv.struct.image.Planar<GrayF32>(GrayF32.class,1,1,3);
    int[][] hsvMatch                                     = new int[1][1];
    int[][] runlengthMatch                               = new int[1][1];
    int[][] combinedOutput                               = new int[1][1];
    int[][] free_hist                                    = new int[1][1];
    int free_hist_window_length                          = 5;
    int free_hist_window_average                         = 3;
    int[][] free_space_frozen                            = new int[1][1];
    boofcv.struct.image.GrayF32 image                    = new boofcv.struct.image.GrayF32();  // TODO - hardcoded
    int previewRotation = 0;                                // TODO - hardcoded
    boolean flipHorizontal = false;                         // TODO - hardcoded

    byte[] current_image_bytes;

    enum typeOfImage {
        GRAY, HSV
    }

//    @Override
    public void convertPreviewForBoofCV(byte[] bytes, Camera camera) {
            Log.i("convertPreviewForBoofCV", "started convertPreviewForBoofCV(last_frame_bytes(), camera);");
        convertPreviewForBoofCV(bytes, camera, typeOfImage.GRAY);
            Log.i("convertPreviewForBoofCV", "finished convertPreviewForBoofCV(last_frame_bytes(), camera);");
    }

    public void convertPreviewBoofCVHsv(byte[] bytes, Camera camera) {
        convertPreviewForBoofCV(bytes, camera, typeOfImage.HSV);
    }

    public void convertPreviewForBoofCV(byte[] bytes, Camera camera, typeOfImage imageType_) {  // TODO - Strategy Pattern / refactor for Gray (fiducials, SURF, etc) vs HSV (free space, etc)
        current_image_bytes = bytes;                                                                // TODO - Strategy Pattern - is common
//        if( thread == null )
//            return;
        if(null == matGray) {
            Log.w(TAG,"convertPreviewForBoofCV: null == matGray: returning.");
            return;
        }
        if(null == bytes) {
            Log.w(TAG,"convertPreviewForBoofCV: null == bytes: returning.");
            return;
        }
        int width_  = (int)Math.ceil(matGray.size().width);
        int height_ = (int)Math.ceil(matGray.size().height);
        image = image.createNew(width_,height_);

        if( typeOfImage.GRAY == imageType_) {
            Log.i(TAG,"convertPreviewForBoofCV: image size: ConvertNV21.nv21ToGray(image.width="+image.width+", image.height="+image.height+", image, image.getClass()="+image.getClass().getCanonicalName()+")");
            ConvertNV21.nv21ToGray(bytes, image.width, image.height, (ImageGray) image, (Class) image.getClass());
//        synchronized ( lockConvert ) {
//            if( imageType_.getFamily() == GRAY ) {
//                ConvertNV21.nv21ToGray(bytes, image.width, image.height, (ImageGray) image,(Class) image.getClass());
//            } else if( imageTypeGray.getFamily() == ImageType.Family.PLANAR ) {
//                if (imageTypeGray.getDataType() == ImageDataType.U8)
//                    ConvertNV21.nv21ToMsRgb_U8(bytes, image.width, image.height, (Planar) image);
//                else if (imageTypeGray.getDataType() == ImageDataType.F32)
//                    ConvertNV21.nv21ToMsRgb_F32(bytes, image.width, image.height, (Planar) image);
//                else
//                    throw new RuntimeException("Oh Crap");
//            } else if( imageTypeGray.getFamily() == ImageType.Family.INTERLEAVED ) {
//                if( imageTypeGray.getDataType() == ImageDataType.U8)
//                    ConvertNV21.nv21ToInterleaved(bytes, image.width, image.height, (InterleavedU8) image);
//                else if( imageTypeGray.getDataType() == ImageDataType.F32)
//                    ConvertNV21.nv21ToInterleaved(bytes, image.width, image.height, (InterleavedF32) image);
//                else
//                    throw new RuntimeException("Oh Crap");
        } else if(typeOfImage.HSV == imageType_) {
            Log.i(TAG,"convertPreviewForBoofCV: image size: HSV: image.width="+image.width+", image.height="+image.height);
            if(null == imageRgb || imageRgb.width!=image.width || imageRgb.height!=image.height) {
                imageRgb = new boofcv.struct.image.Planar<GrayF32>(GrayF32.class,image.width,image.height,3);
            }
            if(null == imageHsv || imageHsv.width!=image.width || imageHsv.height!=image.height) {
                imageHsv = new boofcv.struct.image.Planar<GrayF32>(GrayF32.class,image.width,image.height,3);
            }
        hsvMatch          = resize(hsvMatch, image.width, image.height);                        // TODO - Strategy Pattern for Gray/HSV and/or move this initialisation to the free space code
//                if(null == hsvMatch) {
//                    hsvMatch = new int[image.width][image.height];
//                } else if( hsvMatch.length!=image.width || hsvMatch[0].length!=image.height) {
//                    hsvMatch = new int[image.width][image.height];
//                }
        runlengthMatch    = resize(runlengthMatch, image.width, image.height);                  // TODO - Strategy Pattern for Gray/HSV and/or move this initialisation to the free space code
//                if(null == runlengthMatch) {
//                    runlengthMatch = new int[image.width][image.height];
//                } else if( runlengthMatch.length!=image.width || runlengthMatch[0].length!=image.height) {
//                    runlengthMatch = new int[image.width][image.height];
//                }
        combinedOutput    = reset(combinedOutput, image.width, image.height);                   // TODO - Strategy Pattern for Gray/HSV and/or move this initialisation to the free space code
        free_hist         = resize(free_hist, image.width, image.height);                       // TODO - Strategy Pattern for Gray/HSV and/or move this initialisation to the free space code
        free_space_frozen = resize(free_space_frozen, image.width, image.height);               // TODO - Strategy Pattern for Gray/HSV and/or move this initialisation to the free space code
        ConvertNV21.nv21ToMsRgb_F32(bytes, image.width, image.height, imageRgb);
            Log.i(TAG,"convertPreviewForBoofCV: image size: ConvertNV21.nv21ToMsRgb_F32: image.width="+image.width+", image.height="+image.height);
        ColorHsv.rgbToHsv_F32(imageRgb, imageHsv);
            Log.i(TAG,"convertPreviewForBoofCV: image size: ColorHsv.rgbToHsv_F32: image.width="+image.width+", image.height="+image.height);
        } else {
            throw new RuntimeException("Unexpected image type: "+ imageTypeGray);
        }

        if( previewRotation == 180 ) {                                                          // TODO - Strategy Pattern - is common
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

    private int[][] resize(int[][] arrayToResize_, int image_width_, int image_height_) {
        if(null == arrayToResize_) {
            arrayToResize_ = new int[image_width_][image_height_];
        } else if( arrayToResize_.length!=image_width_|| arrayToResize_[0].length!=image_height_) {
            arrayToResize_ = new int[image_width_][image_height_];
        }
        return arrayToResize_;
    }

    private int[][] reset(int[][] arrayToReset_, int image_width_, int image_height_) {
        arrayToReset_ = new int[image_width_][image_height_];
        return arrayToReset_;
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

    private class CalcImageDimensions {
        private int width;
        private int height;

        public int getWidth() {
            return width;
        }

        public int getHeight() {
            return height;
        }

        public CalcImageDimensions() {
            width = new Double(matGray.size().width).intValue();
            height = new Double(matGray.size().height).intValue();
            px_pixels = (float) (matGray.size().width / 2.0);
            py_pixels = (float) (matGray.size().height / 2.0);
        }
        
        private float px_pixels;
        private float py_pixels;

        public float getPx_pixels() {
            return px_pixels;
        }

        public float getPy_pixels() {
            return py_pixels;
        }
        
        public double getSkew() {
            return 0.0;
        }
    }

    private class MarkerIdFormatValidator {
        private boolean isValid;
        private FiducialDetector<GrayF32> detector;
        private int i;
        private int tag_id;

        public MarkerIdFormatValidator(FiducialDetector<GrayF32> detector, int i, int tag_id) {
            this.detector = detector;
            this.i = i;
            this.tag_id = tag_id;
            this.isValid= true;
            if( detector.hasUniqueID() ) {
//                System.out.println("Target ID = " + detector.getId(i));
            long tag_id_long = detector.getId(i);
//                tag_id = (int)tag_id_long;
//                if ((long)tag_id != tag_id_long) {
//                    //throw new IllegalArgumentException(l + " cannot be cast to int without changing its value.");
//                    System.out.println(" BoofCV: cannot use tag: tag_id_long '"+tag_id_long+"' cannot be cast to int without changing its value.");
//                    isValid = false;
//                }
            }
        }

        boolean isValid() {
            return isValid;
        }

        public int getTag_id() {
            return tag_id;
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
//            // swap gray buffers so that convertPreviewForBoofCV is modifying the copy which is not in use
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



    static double[] free_space_white              = new double[]{ 255d, 255d, 255d, 125d};
    static double[] free_space_orange             = new double[]{ 255d, 125d,   0d, 255d};
    static double[] free_space_gold               = new double[]{ 255d, 215d,   0d, 255d};
    static double[] free_space_darkGreen          = new double[]{   0d, 100d,   0d, 255d};
    static double[] free_space_paleGreen          = new double[]{ 152d, 251d, 152d, 255d};
    static double[] free_space_magenta            = new double[]{ 255d,   0d, 255d, 255d};
    static double[] free_space_projected_darkCyan = new double[]{   0d, 139d, 139d, 255d};
    static double[] free_space_projected_mediumOrchid = new double[]{ 153d,  50d, 204d, 255d};
    static double[] free_space_yellow             = new double[]{ 125d, 125d,   0d, 255d};
    static double[] free_space_sienna             = new double[]{ 160d,  82d,  45d, 255d};
    static double[] free_space_projected_purple   = new double[]{   0d, 125d, 125d, 255d};
    static double[] free_space_projected_aqua     = new double[]{ 127d, 255d, 212d, 255d};

    /**
     * */
    private void projectOntoWorldFreeSpace(final int x_pixels, final int y_pixels, final int max_width, final int max_height, final double pose_z, Mat matRgb, double[] gold_, double[] purple_) {
        //  CameraPinhole pinholeModel = new CameraPinhole(focalLengthCalc.getFocal_length_in_pixels_x(), focalLengthCalc.getFocal_length_in_pixels_y(), calcImgDim.getSkew(), calcImgDim.getPx_pixels(), calcImgDim.getPy_pixels(), calcImgDim.getWidth(), calcImgDim.getHeight());
        FocalLengthCalculator focalLengthCalculator = new FocalLengthCalculator();
        CalcImageDimensions calcImgDim = new CalcImageDimensions();
        float  fx   = focalLengthCalculator.getFocal_length_in_pixels_x(matGray.width());
        float  fy   = focalLengthCalculator.getFocal_length_in_pixels_y(matGray.height());
        float  u0   = calcImgDim.getPx_pixels();
        float  v0   = calcImgDim.getPy_pixels();
        double skew = calcImgDim.getSkew();
        int    img_width_px  = calcImgDim.getWidth();
        int    img_height_px = calcImgDim.getHeight();

        PinholeCamera camera = new PinholeCamera();
        double[] pixel_to_paint = camera.project_pixel_to_world(x_pixels, y_pixels, max_width, max_height, pose_z, fx, fy, u0, v0);
        int    y_pixel_to_paint = (int)pixel_to_paint[0];  // image height
        int    x_pixel_to_paint = (int)pixel_to_paint[1];  // image width
        double projected_distance =    pixel_to_paint[2];  // real-world distance along optical axis: using robotics x-forward y-left -z-up coordinate system
        double projected_y      =      pixel_to_paint[3];  // real-world distance along horizontal axis orthogonal to optical axis: left is +ve : using robotics x-forward y-left -z-up coordinate system

        if(x_pixel_to_paint < 0.0d || x_pixel_to_paint > max_width || y_pixel_to_paint < 0.0d || y_pixel_to_paint > max_height) {
            Log.i("projectOntoWorld","out of bounds for projection: x_pixels="+x_pixels+",y_pixels="+y_pixels+" : y_pixel="+y_pixel_to_paint+",x_pixel="+x_pixel_to_paint+", projected_distance="+projected_distance+", projected_y="+projected_y);
            matRgb.put(y_pixels, x_pixels, free_space_sienna);
            return;
        } else {
            Log.i("projectOntoWorld","projection: x_pixels="+x_pixels+",y_pixels="+y_pixels+" : y_pixel="+y_pixel_to_paint+",x_pixel="+x_pixel_to_paint+", projected_distance="+projected_distance+", projected_y="+projected_y);
            matRgb.put(y_pixels, x_pixels, gold_);                                // repaint from white to gold
            matRgb.put(y_pixel_to_paint, x_pixel_to_paint, purple_);    // project in purple on top
        }                                // TODO - boolean output for the block
    }


    private class BoofCVDetectorAndReporter {
        private boolean myResult;
        private HashMap<RobotId, List<DetectedTag>> robotsDetected_;
        private RobotId singleDummyRobotId_;
        private List<DetectedTag> landmarkFeatures;
        private Date imageFrameTime;
        private FiducialDetector<GrayF32> detector;
        private boolean smartCameraExtrinsicsCalibrator_first_notification_this_frame;
        private boolean detect_all_markers_in_frame;
        private int detectionOrder_;
        private int tag_id;
        private VisionTask visionTask;
        private List<VisionTask> visionTaskListToExecute;
        private String logTagTag;
        private Se3_F64 targetToSensor_boofcvFrame;
        private PixelPosition pixelPosition;

        public BoofCVDetectorAndReporter(HashMap<RobotId, List<DetectedTag>> robotsDetected_, RobotId singleDummyRobotId_, List<DetectedTag> landmarkFeatures, Date imageFrameTime, FiducialDetector<GrayF32> detector, boolean smartCameraExtrinsicsCalibrator_first_notification_this_frame, boolean DETECT_ALL_MARKERS_IN_FRAME, int detectionOrder_, int tag_id, VisionTask visionTask, List<VisionTask> visionTaskListToExecute, String logTagTag, Se3_F64 targetToSensor_boofcvFrame, PixelPosition pixelPosition) {
            this.robotsDetected_ = robotsDetected_;
            this.singleDummyRobotId_ = singleDummyRobotId_;
            this.landmarkFeatures = landmarkFeatures;
            this.imageFrameTime = imageFrameTime;
            this.detector = detector;
            this.smartCameraExtrinsicsCalibrator_first_notification_this_frame = smartCameraExtrinsicsCalibrator_first_notification_this_frame;
            detect_all_markers_in_frame = DETECT_ALL_MARKERS_IN_FRAME;
            this.detectionOrder_ = detectionOrder_;
            this.tag_id = tag_id;
            this.visionTask = visionTask;
            this.visionTaskListToExecute = visionTaskListToExecute;
            this.logTagTag = logTagTag;
            this.targetToSensor_boofcvFrame = targetToSensor_boofcvFrame;
            this.pixelPosition = pixelPosition;
        }

        boolean is() {
            return myResult;
        }

        public boolean isSmartCameraExtrinsicsCalibrator_first_notification_this_frame() {
            return smartCameraExtrinsicsCalibrator_first_notification_this_frame;
        }

        public BoofCVDetectorAndReporter invoke() {

            System.out.println("onCameraFrame: 3D Location: start for this tag "+tag_id+"---------------------------------------------------------");
            Vector3D_F64 transBoofCV_TtoS = targetToSensor_boofcvFrame.getTranslation();
            Quaternion_F64 quatBoofCV_TtoS = transform_to_quaternion_boofcv(targetToSensor_boofcvFrame);
            System.out.println("onCameraFrame: 3D Location: targetToSensor_boofcvFrame : BoofCV frame "
                    +tag_id+": x = " + transBoofCV_TtoS.getX() + ", y = " + transBoofCV_TtoS.getY() + ", z = " + transBoofCV_TtoS.getZ());
            System.out.println("onCameraFrame: 3D Location: targetToSensor_boofcvFrame : BoofCV frame "
                    +tag_id+": qx = " + quatBoofCV_TtoS.x + ", qy = " + quatBoofCV_TtoS.y + ", qz = " + quatBoofCV_TtoS.z + ", qw = " + quatBoofCV_TtoS.w);
            double x = transBoofCV_TtoS.getX();  double y = transBoofCV_TtoS.getY();  double z = transBoofCV_TtoS.getZ();
            double qx = quatBoofCV_TtoS.x;  double qy = quatBoofCV_TtoS.y;  double qz = quatBoofCV_TtoS.z;  double qw = quatBoofCV_TtoS.w;
            System.out.println("onCameraFrame: 3D Location: targetToSensor_boofcvFrame : BoofCV frame "
                    +tag_id+": figure(f1); temp__plot_boofcv_StoT_inVOS_rot("+qw+","+qx+","+qy+","+qz+",  "+x+","+y+","+z+")  % "+tag_id);

            DenseMatrix64F rot = targetToSensor_boofcvFrame.getR();
            System.out.println("onCameraFrame: 3D Location: sensorToTarget_boofcvFrame : BoofCV frame : "+tag_id+": rotation matrix= [ "+
                rot.get(0,0) +", "+rot.get(0,1)+", "+rot.get(0,2)+"; "+ //get(row, column)
                rot.get(1,0) +", "+rot.get(1,1)+", "+rot.get(1,2)+"; "+
                rot.get(2,0) +", "+rot.get(2,1)+", "+rot.get(2,2)+" ]");
            System.out.println("onCameraFrame: 3D Location: sensorToTarget_boofcvFrame : BoofCV frame : "+tag_id+": rotation matrix= "
                    +targetToSensor_boofcvFrame.getR());

            Se3_F64 sensorToTarget_boofcvFrame = targetToSensor_boofcvFrame.invert(null);
            Vector3D_F64 sensorToTarget_boofcvFrame_transl = sensorToTarget_boofcvFrame.getTranslation();
            Quaternion_F64 sensorToTarget_boofcvFrame_quat = transform_to_quaternion_boofcv(sensorToTarget_boofcvFrame);
            System.out.println("onCameraFrame: 3D Location: sensorToTarget_boofcvFrame : BoofCV frame "
                    +tag_id+": x = " + sensorToTarget_boofcvFrame_transl.getX() + ", y = " + sensorToTarget_boofcvFrame_transl.getY() + ", z = " + sensorToTarget_boofcvFrame_transl.getZ());
            System.out.println("onCameraFrame: 3D Location: sensorToTarget_boofcvFrame : BoofCV frame "
                    +tag_id+": qx = " + sensorToTarget_boofcvFrame_quat.x + ", qy = " + sensorToTarget_boofcvFrame_quat.y + ", qz = " + sensorToTarget_boofcvFrame_quat.z + ", qw = " + sensorToTarget_boofcvFrame_quat.w);



            System.out.println("onCameraFrame: 3D Location: end for this tag "+tag_id+"---------------------------------------------------------");


            // testing 2017_08_23
            double[] eulerZYZ = new double[]{0, 0, 0};
            ConvertRotation3D_F64.matrixToEuler(targetToSensor_boofcvFrame.getR(), EulerType.ZYZ, eulerZYZ);
            Log.i(logTagTag, "onCameraFrame : testing 2017_08_23: eulerZYZ = " + eulerZYZ[0] + "," + eulerZYZ[1] + "," + eulerZYZ[2]);

            Se3_F64 sensorToTarget_testing = null;
            sensorToTarget_testing = targetToSensor_boofcvFrame.invert(sensorToTarget_testing);

            Quaternion_F64 sensorToTarget_testing_quat;
            sensorToTarget_testing_quat = new Quaternion_F64();
            ConvertRotation3D_F64.matrixToQuaternion(sensorToTarget_testing.getRotation(), sensorToTarget_testing_quat);
//
//                            detectedFeaturesClient.reportDetectedFeature(80000+tag_id,
//                                    sensorToTarget_testing.getZ(), sensorToTarget_testing.getX(), sensorToTarget_testing.getY(),
//                                    sensorToTarget_testing_quat.z,sensorToTarget_testing_quat.x,sensorToTarget_testing_quat.y,sensorToTarget_testing_quat.w);

            /** Mirror across YZ plane / mirror along X axis:
             * sensor-to-target +x = target-to-sensor +z
             * sensor-to-target +y = target-to-sensor +x
             * sensor-to-target +z = target-to-sensor +y   */
            Se3_F64 targetToSensor_boofcvFrame_testing = new Se3_F64();
            detector.getFiducialToCamera(detectionOrder_, targetToSensor_boofcvFrame_testing);
            /** Convert from BoofCV coordinate convention to ROS coordinate convention */
            Se3_F64 targetToSensor_ROSFrame = new Se3_F64();
            targetToSensor_ROSFrame.setTranslation(new Vector3D_F64(
                    targetToSensor_boofcvFrame_testing.getZ(), targetToSensor_boofcvFrame_testing.getX(), targetToSensor_boofcvFrame_testing.getY()));


            Quaternion_F64 tToS_Boof_testing_quat = ConvertRotation3D_F64.matrixToQuaternion(targetToSensor_boofcvFrame_testing.getRotation(), null);
            Quaternion_F64 tToS_ROS_testing_quat = new Quaternion_F64(  /** new Quaternion_F64(w, x, y, z) */
                    tToS_Boof_testing_quat.w, tToS_Boof_testing_quat.z, tToS_Boof_testing_quat.x, tToS_Boof_testing_quat.y);
            Quaternion_F64 sensorToTarget_ROSFrame_mirrored_q = new Quaternion_F64(  /** new Quaternion_F64(w, x, y, z) */
                    tToS_ROS_testing_quat.w, tToS_ROS_testing_quat.x, -tToS_ROS_testing_quat.y, -tToS_ROS_testing_quat.z
            );
            DenseMatrix64F sensorToTarget_ROSFrame_mirrored_rot = quaternionToNewMAtrix(sensorToTarget_ROSFrame_mirrored_q);

            /** Mirror across the XY plane. */
            Se3_F64 sensorToTarget_ROSFrame_mirrored = new Se3_F64();
            sensorToTarget_ROSFrame_mirrored.setTranslation(new Vector3D_F64(
                    targetToSensor_ROSFrame.getX(), -targetToSensor_ROSFrame.getY(), -targetToSensor_ROSFrame.getZ()));

            sensorToTarget_ROSFrame_mirrored.setRotation(sensorToTarget_ROSFrame_mirrored_rot);

            DenseMatrix64F sensorToTarget_ROSFrame_mirrored_rot_rotate_around_X_by_180_rot = new DenseMatrix64F(3, 3);

            DenseMatrix64F rotate_around_X_by_180 = CommonOps.identity(3);
            ConvertRotation3D_F64.setRotX(PI, rotate_around_X_by_180);
//                        CommonOps.mult(rotate_around_X_by_180,sensorToTarget_ROSFrame_mirrored_rot,sensorToTarget_ROSFrame_mirrored_rot_rotate_around_X_by_180_rot);
            /** Note: post-multiply with BoofCV - I think that it is column-major ?? */  // TODO - check BoofCV conventions
            CommonOps.mult(sensorToTarget_ROSFrame_mirrored_rot, rotate_around_X_by_180, sensorToTarget_ROSFrame_mirrored_rot_rotate_around_X_by_180_rot);
            Quaternion_F64 sensorToTarget_ROSFrame_mirrored_rotate_around_X_by_180_q = new Quaternion_F64();
            ConvertRotation3D_F64.matrixToQuaternion(sensorToTarget_ROSFrame_mirrored_rot_rotate_around_X_by_180_rot, sensorToTarget_ROSFrame_mirrored_rotate_around_X_by_180_q);


//                      /** This is the camera-to-marker transform */ - keep this code, but publish the camera-to-robot-base transform instead
//                      /** Report as e.g. 60170, 60155, etc. */
            for(VisionTask visionTaskInList : visionTaskListToExecute) {
                detectedFeaturesClient.reportDetectedFeature(visionTaskInList.getRobotId(), visionTaskInList.getRequestId(),
                        60000 + tag_id,
                        // TODO - use this - int tag_id_reported = MARKER_OFFSET_INT+tag_id;
                        sensorToTarget_ROSFrame_mirrored.getX(), sensorToTarget_ROSFrame_mirrored.getY(), sensorToTarget_ROSFrame_mirrored.getZ(),
                        sensorToTarget_ROSFrame_mirrored_rotate_around_X_by_180_q.x, sensorToTarget_ROSFrame_mirrored_rotate_around_X_by_180_q.y, sensorToTarget_ROSFrame_mirrored_rotate_around_X_by_180_q.z, sensorToTarget_ROSFrame_mirrored_rotate_around_X_by_180_q.w);

                if(RECORD_DATA) {
                    Log.i(logTagTag, "detectAndEstimate_BoofCV_Fiducial: recording detection data: start.");
                    printlnToFile("\ntag_id:"+tag_id+" [ "+pixelPosition.getU()+" , "+pixelPosition.getV()+" ] ", imageFrameTime);
                    Se3_F64 trans = sensorToTarget_ROSFrame_mirrored;
                    printlnToFile("task: {robotId:" + visionTaskInList.getRobotId()+ ", featureDescriptor: " + tag_id + " }, pose: { x: " + trans.getX() + ", y:" + trans.getY() + ", z:" + trans.getZ() + " }", imageFrameTime);
                    RECORD_DATA_RECORDED = true;
                    Log.i(logTagTag, "detectAndEstimate_BoofCV_Fiducial: recording detection data: end.");
                } else {
                    Log.i(logTagTag, "detectAndEstimate_BoofCV_Fiducial: not recording detection data.");
                }
            }


            Se3_F64 sensorToTarget_ROSFrame_mirrored_rot_rotate_around_X_by_180_t = new Se3_F64();
            sensorToTarget_ROSFrame_mirrored_rot_rotate_around_X_by_180_t.setRotation(sensorToTarget_ROSFrame_mirrored_rot_rotate_around_X_by_180_rot);
            sensorToTarget_ROSFrame_mirrored_rot_rotate_around_X_by_180_t.setTranslation(sensorToTarget_ROSFrame_mirrored.getTranslation());


            geometry_msgs.Point position = visionTask.getRelationToBase().getPosition();
            Se3_F64 transformOfFeatureInVisualModel = new Se3_F64();                    // transform from robot to marker, e.g. base_link to feature
            transformOfFeatureInVisualModel.setTranslation(position.getX(), position.getY(), position.getZ());
            Quaternion_F64 rotationOfFeatureInVisualModel_q = convertRosToBoofcvQuaternion(visionTask);
            DenseMatrix64F rotationOfFeatureInVisualModel_m = quaternionToNewMAtrix(rotationOfFeatureInVisualModel_q);
            transformOfFeatureInVisualModel.setRotation(ConvertRotation3D_F64.quaternionToMatrix(rotationOfFeatureInVisualModel_q, rotationOfFeatureInVisualModel_m));
            Se3_F64 transformOfFeatureInVisualModel_inv = new Se3_F64();
            transformOfFeatureInVisualModel.invert(transformOfFeatureInVisualModel_inv); // transform from marker to robot

            if(smartCameraExtrinsicsCalibrator_first_notification_this_frame || detect_all_markers_in_frame) {
                smartCameraExtrinsicsCalibrator_first_notification_this_frame = false;

//                        public void recordRobotObservation(String robotId_, java.util.Date imageFrameTime_, PixelPosition pixelPosition_, Se3_F64 transformOfFeatureInVisualModel_)
                Log.i(TAG,"before smartCameraExtrinsicsCalibrator.recordRobotObservation");
                smartCameraExtrinsicsCalibrator.recordRobotObservation(
                        singleDummyRobotId_.idString(), imageFrameTime, pixelPosition, transformOfFeatureInVisualModel);
                Log.i(TAG,"after smartCameraExtrinsicsCalibrator.recordRobotObservation");
                Log.i(TAG,"before smartCameraExtrinsicsCalibrator.estimateExtrinsicsFromObservations()");
                Se3_F64 extrinsics = smartCameraExtrinsicsCalibrator.estimateExtrinsicsFromObservations(matGray.width(), matGray.height());
                Log.i(TAG,"after smartCameraExtrinsicsCalibrator.estimateExtrinsicsFromObservations(): extrinsics="+extrinsics);

                //  askForPose_then_recordDetection(imageFrameTime, transformOfFeatureInVisualModel, pixelPosition);
            }

            DenseMatrix64F sensorToTarget_ROSFrame_toRobotBaseLink_rot = new DenseMatrix64F(3, 3);

            Se3_F64 sensorToTarget_ROSFrame_toRobotBaseLink = new Se3_F64();

//                      this is the camera-to-robot-base transform
//                      /** Report as e.g. 60170, 60155, etc. */
            transformOfFeatureInVisualModel_inv.concat(                                 // pose from previous as transform from camera to marker
                    sensorToTarget_ROSFrame_mirrored_rot_rotate_around_X_by_180_t,      // transform from marker to robot, from robot visual model
                    sensorToTarget_ROSFrame_toRobotBaseLink);                          // output
            Quaternion_F64 sensorToTarget_ROSFrame_toRobotBaseLink_q = new Quaternion_F64();
            ConvertRotation3D_F64.matrixToQuaternion(sensorToTarget_ROSFrame_toRobotBaseLink.getRotation(), sensorToTarget_ROSFrame_toRobotBaseLink_q);

            for(VisionTask visionTaskInList: visionTaskListToExecute) {
                detectedFeaturesClient.reportDetectedFeature(visionTaskInList.getRobotId(), visionTaskInList.getRequestId(),
                        70000 + tag_id,
                        sensorToTarget_ROSFrame_toRobotBaseLink.getX(), sensorToTarget_ROSFrame_toRobotBaseLink.getY(), sensorToTarget_ROSFrame_toRobotBaseLink.getZ(),
                        sensorToTarget_ROSFrame_toRobotBaseLink_q.x, sensorToTarget_ROSFrame_toRobotBaseLink_q.y, sensorToTarget_ROSFrame_toRobotBaseLink_q.z, sensorToTarget_ROSFrame_toRobotBaseLink_q.w);
            }
            //  Report the detected feature pose in the world coordinate frame/system,
            // applying the camera's current pose to the detected feature's pose,
            // before reporting to the VOS Server.

            if (poseKnown()) {
                Log.i(logTagTag, "poseKnown()");
                Log.i(logTagTag, "poseKnown(): translation: " + MainActivity.this.position[0] + "," + MainActivity.this.position[1] + "," + MainActivity.this.position[2]);
                Log.i(logTagTag, "poseKnown(): rotation quaternion: " + MainActivity.this.orientation[0] + "," + MainActivity.this.orientation[1] + "," + MainActivity.this.orientation[2] + "," + MainActivity.this.orientation[3]);
                Se3_F64 worldToCamera = new Se3_F64();
                worldToCamera.setTranslation(MainActivity.this.position[0], MainActivity.this.position[1], MainActivity.this.position[2]);
                Quaternion_F64 worldToCamera_rot_q = new Quaternion_F64(MainActivity.this.orientation[3], MainActivity.this.orientation[0], MainActivity.this.orientation[1], MainActivity.this.orientation[2]);
                DenseMatrix64F worldToCamera_rot_m = new DenseMatrix64F(3, 3);
                worldToCamera.setRotation(
                        ConvertRotation3D_F64.quaternionToMatrix(
                                worldToCamera_rot_q,  //  (double w, double x, double y, double z)
                                worldToCamera_rot_m));

                System.out.println("poseKnown: sending id=" + (40000 + tag_id)
                        +", x="+worldToCamera.getX()+", y="+worldToCamera.getY()+",z="+ worldToCamera.getZ()
                        +",  qx="+worldToCamera_rot_q.x+", qy="+worldToCamera_rot_q.y+", qz="+worldToCamera_rot_q.z+", qw="+worldToCamera_rot_q.w);

                for(VisionTask visionTaskInList: visionTaskListToExecute) {
                    detectedFeaturesClient.reportDetectedFeature(visionTaskInList.getRobotId(), visionTaskInList.getRequestId(),
                            40000 + tag_id,
                            worldToCamera.getX(), worldToCamera.getY(), worldToCamera.getZ(),
                            worldToCamera_rot_q.x, worldToCamera_rot_q.y, worldToCamera_rot_q.z, worldToCamera_rot_q.w);
                }
                Se3_F64 worldToRobotBaseLink = new Se3_F64();
                sensorToTarget_ROSFrame_toRobotBaseLink.concat(
                        worldToCamera,
                        worldToRobotBaseLink
                );
                Quaternion_F64 worldToRobotBaseLink_q = new Quaternion_F64();
                ConvertRotation3D_F64.matrixToQuaternion(worldToRobotBaseLink.getRotation(), worldToRobotBaseLink_q);

                for(VisionTask visionTaskInList: visionTaskListToExecute) {
                    detectedFeaturesClient.reportDetectedFeature(visionTaskInList.getRobotId(), visionTaskInList.getRequestId(),
                            50000 + tag_id,
                            worldToRobotBaseLink.getX(), worldToRobotBaseLink.getY(), worldToRobotBaseLink.getZ(),
                            worldToRobotBaseLink_q.x, worldToRobotBaseLink_q.y, worldToRobotBaseLink_q.z, worldToRobotBaseLink_q.w);

                    System.out.println("poseKnown: sending id=" + (50000 + tag_id)
                            +", x="+worldToRobotBaseLink.getX()+", y="+worldToRobotBaseLink.getY()+",z="+ worldToRobotBaseLink.getZ()
                            +",  qx="+worldToRobotBaseLink_q.x+", qy="+worldToRobotBaseLink_q.y+", qz="+worldToRobotBaseLink_q.z+", qw="+worldToRobotBaseLink_q.w);
                }

            } else {
                Log.i(logTagTag, "! poseKnown()");
            }


            Se3_F64 translation_to_marker = sensorToTarget_ROSFrame_mirrored;
            Quaternion_F64 quaternion_to_marker = sensorToTarget_ROSFrame_mirrored_rotate_around_X_by_180_q;

            double[] eulerZYZ_fromInvert = new double[]{0, 0, 0};
            ConvertRotation3D_F64.matrixToEuler(sensorToTarget_testing.getR(), EulerType.ZYZ, eulerZYZ_fromInvert);
            Log.i(logTagTag, "onCameraFrame : testing 2017_08_23: eulerZYZ_fromInvert = " + eulerZYZ_fromInvert[0] + "," + eulerZYZ_fromInvert[1] + "," + eulerZYZ_fromInvert[2]);
            Vector3D_F64 sensorToTarget_testing_trans = sensorToTarget_testing.getTranslation();
            Log.i(logTagTag, "onCameraFrame: testing 2017_08_23: sensorToTarget_testing_trans : x = " + sensorToTarget_testing_trans.getX() + ", y = " + sensorToTarget_testing_trans.getY() + ", z = " + sensorToTarget_testing_trans.getZ());
            sensorToTarget_testing_quat = new Quaternion_F64();
            ConvertRotation3D_F64.matrixToQuaternion(sensorToTarget_testing.getR(), sensorToTarget_testing_quat);
            Log.i(logTagTag, "onCameraFrame: testing 2017_08_23: sensorToTarget_testing_quat : qx = " + sensorToTarget_testing_quat.x + ", qy = " + sensorToTarget_testing_quat.y + ", qz = " + sensorToTarget_testing_quat.z + ", qw = " + sensorToTarget_testing_quat.w);

            if (isPartOfRobotVisualModel(tag_id)) {
                List<DetectedTag> visionTaskFeaturesDetected = visionTaskFeaturesDetected(robotsDetected_, singleDummyRobotId_);
                DetectedTag detectedTag = new DetectedTag(tag_id, translation_to_marker, quaternion_to_marker);
                visionTaskFeaturesDetected.add(detectedTag);
                Log.i(logTagTag, "onCameraFrame: isPartOfRobotVisualModel TAG - tag_id " + tag_id + " - 2D Image Location = " + pixelPosition);
            } else if (isALandmark(tag_id)) {
                DetectedTag detectedTag = new DetectedTag(tag_id, translation_to_marker, new Point2D_F64(pixelPosition.getU(),pixelPosition.getV()));
                landmarkFeatures.add(detectedTag);
                Log.i(logTagTag, "onCameraFrame: isALandmark TAG - tag_id " + tag_id + " landmarkFeatures.size()=" + landmarkFeatures.size() + " - 2D Image Location = " + pixelPosition);
            } else { // not part of something that we are looking for, so ignore
                Log.i(logTagTag, "onCameraFrame: IGNORING TAG - not part of robot visual model - tag_id " + tag_id + " - 2D Image Location = " + pixelPosition);
                myResult = true;
                return this;
            }

            //// TODO - timing here  c[camera_num]-f[frameprocessed]-detectionOrder_[iteration]-t[tagid]
            Log.i(logTagTag, "onCameraFrame: after detectedFeaturesClient.reportDetectedFeature");
            if (LOCALISING_CAMERA_FROM_OBSERVED_FEATURES) {
                updateLocationFromDetectedFeature(tag_id, logTagTag, translation_to_marker, quaternion_to_marker);
            }
            if (TESTING_TRANSFORMATIONS_OF_TRANSFORMS) {
                variousUnusedAttemptsAtCoordinateSystemCorrection();
            }
            myResult = false;
            return this;
        }
    }

    @NonNull
    private DenseMatrix64F quaternionToNewMAtrix(Quaternion_F64 quaternion) {
        DenseMatrix64F matrix = new DenseMatrix64F(3, 3);
        ConvertRotation3D_F64.quaternionToMatrix(quaternion, matrix);
        return matrix;
    }
}

