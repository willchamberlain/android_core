package william.chamberlain.androidvosopencvros;

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
import android.view.SurfaceView;
import android.view.Window;
import android.widget.Toast;

import org.ejml.data.DenseMatrix64F;
import org.ejml.data.RowD1Matrix64F;
import org.ejml.ops.CommonOps;
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

import java.net.URI;
import java.util.Arrays;
import java.util.List;
import java.util.regex.Matcher;


import android.util.Log;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;

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
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Quaternion_F64;
import sensor_msgs.Imu;
import william.chamberlain.androidvosopencvros.device.DimmableScreen;
import william.chamberlain.androidvosopencvros.device.ImuCallback;
import william.chamberlain.androidvosopencvros.monitoring.ImuMonitoringPublisher;
import william.chamberlain.androidvosopencvros.resilient.ResilientNetworkActivity;

import static boofcv.struct.image.ImageDataType.F32;
import static boofcv.struct.image.ImageType.Family.GRAY;
import static java.lang.Math.PI;
import static java.lang.Math.tan;
import static william.chamberlain.androidvosopencvros.Constants.APRIL_TAGS_KAESS_36_H_11;
import static william.chamberlain.androidvosopencvros.Constants.tagSize_metres;
import static william.chamberlain.androidvosopencvros.DataExchange.tagPattern_trans_quat;

/**
 * @author chadrockey@gmail.com (Chad Rockey)
 * @author axelfurlan@gmail.com (Axel Furlan)
 */


public class MainActivity
        extends         RosActivity
        implements      CameraBridgeViewBase.CvCameraViewListener2,
            PosedEntity,  // Camera has a pose in the world; defaults to aligned with the map coordinate frame origin and axes.
            DetectedFeaturesHolder,
        ImuCallback,
        DimmableScreen, VariableResolution, VisionSource,
        ResilientNetworkActivity {

    private static final boolean running_native = false;


    private static final String TAG = "vos_aa1::MainActivity";
    public static final String MARKER_NAMESPACE = "apriltags_marker_publisher/tag_markers";

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


    private ImuMonitoringPublisher imu_pub;
    private DetectedFeaturesClient detectedFeaturesClient;
    private SetPoseServer setPoseServer;
    private VisionSourceManagementListener visionSourceManagementListener;
    private LocaliseFromAFeatureClient localiseFromAFeatureClient;
    private RegisterVisionSourceClient registerVisionSourceClient;
    private LocaliseFromAFeatureServer localiseFromAFeatureServer;

    private LocationManager mLocationManager;
    private SensorManager mSensorManager;

    private boolean runImageProcessing = false;
    private boolean displayRgb = true;

    FeatureDataRecorderModeller featureDataRecorderModeller = new FeatureDataRecorderModeller();


    public MainActivity() {
        super("ROS Sensors Driver", "ROS Sensors Driver");
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

        mLocationManager = (LocationManager)this.getSystemService(Context.LOCATION_SERVICE);
        mSensorManager = (SensorManager)this.getSystemService(SENSOR_SERVICE);
        //  cameraManager().getCameraCharacteristics();  -- requires API 21


        if( running_native) {
            // Load ndk built module, as specified
            // in moduleName in build.gradle
            System.loadLibrary("native-lib");
            System.loadLibrary("apriltags_kaess");
            System.loadLibrary("apriltags_umich");
            getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
            setContentView(R.layout.activity_main);
        }

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
        System.out.println("MainActivity: onCreate: before running _cameraBridgeViewBase.setMaxFrameSize(640,480)");
        _cameraBridgeViewBase.setMaxFrameSize(320,240);  // http://stackoverflow.com/questions/17868954/android-opencv-how-to-set-camera-resolution-when-using-camerabridgeviewbase
        System.out.println("MainActivity: onCreate: after running _cameraBridgeViewBase.setMaxFrameSize(640,480)");
        _cameraBridgeViewBase.setVisibility(SurfaceView.VISIBLE);
        _cameraBridgeViewBase.setCvCameraViewListener(this);

        resolutionMinMax(100,100,400,300);
        Log.i("onCreate", "onCreate: set resolution low");

        if (!OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, _baseLoaderCallback)){  // http://stackoverflow.com/questions/24732504/live-stream-video-processing-in-android-using-opencv
            Log.e("onCreate", "onCreate: Cannot connect to OpenCV Manager");
        }else {
            Log.i("onCreate", "onCreate: opencv successfull");
        }

        if(running_native) {
            tagDetectorPointer = newTagDetectorUmich();
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


    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) // configure nodes: config gets fed to an AsyncTask to start the Nodes in a Bound Service: see https://developer.android.com/reference/android/app/Service.html , https://developer.android.com/guide/components/processes-and-threads.html
    {
        final String NODE_NAMESPACE = Naming.cameraNamespace(getCamNum());
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
        System.out.print("init: masterURI=");System.out.println(masterURI);
        int currentapiVersion = android.os.Build.VERSION.SDK_INT;

        int sensorDelay = 20000; // 20,000 us == 50 Hz for Android 3.1 and above
//        if(currentapiVersion <= android.os.Build.VERSION_CODES.HONEYCOMB){
//            sensorDelay = SensorManager.SENSOR_DELAY_UI; // 16.7Hz for older devices.  They only support enum values, not the microsecond version.
//        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration8 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration8.setMasterUri(masterURI);
            nodeConfiguration8.setNodeName(NODE_NAMESPACE+"detectedfeatures_serviceclient_node");
            this.detectedFeaturesClient = new DetectedFeaturesClient();
            detectedFeaturesClient.setCameraFrameId(Naming.cameraFrameId(getCamNum()));
            detectedFeaturesClient.setPosedEntity(this);
            nodeMainExecutor.execute(this.detectedFeaturesClient, nodeConfiguration8);
        }


        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration.setMasterUri(masterURI);
            nodeConfiguration.setNodeName(NODE_NAMESPACE+"pose_server");
            this.setPoseServer = new SetPoseServer();
            setPoseServer.setNodeNamespace(NODE_NAMESPACE);
            setPoseServer.setPosedEntity(this);
            nodeMainExecutor.execute(this.setPoseServer, nodeConfiguration);
        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration.setMasterUri(masterURI);
            nodeConfiguration.setNodeName(NODE_NAMESPACE+"vision_source_management_topic_listener");
            this.visionSourceManagementListener = new VisionSourceManagementListener();
            visionSourceManagementListener.setNodeNamespace(NODE_NAMESPACE);
            visionSourceManagementListener.setDimmableScreen(this);
            visionSourceManagementListener.setVariableResolution(this);
            visionSourceManagementListener.setVisionSource(this);
            nodeMainExecutor.execute(this.visionSourceManagementListener, nodeConfiguration);
        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration8 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration8.setMasterUri(masterURI);
            nodeConfiguration8.setNodeName(NODE_NAMESPACE+"localiseFromAFeature_serviceclient_node");
            this.localiseFromAFeatureClient = new LocaliseFromAFeatureClient();
            localiseFromAFeatureClient.setCameraFrameId(Naming.cameraFrameId(getCamNum()));
            localiseFromAFeatureClient.setPosedEntity(this);
            nodeMainExecutor.execute(this.localiseFromAFeatureClient, nodeConfiguration8);
        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration8 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration8.setMasterUri(masterURI);
            nodeConfiguration8.setNodeName(NODE_NAMESPACE+"registervisionsource_serviceclient_node");
            this.registerVisionSourceClient = new RegisterVisionSourceClient();
            registerVisionSourceClient.setBaseUrl(Naming.cameraNamespace(getCamNum()));
            nodeMainExecutor.execute(this.registerVisionSourceClient, nodeConfiguration8);
        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration8 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration8.setMasterUri(masterURI);
            nodeConfiguration8.setNodeName(NODE_NAMESPACE+"localiseFromAFeature_serviceserver_node");
            this.localiseFromAFeatureServer = new LocaliseFromAFeatureServer();
            localiseFromAFeatureServer.setNodeNamespace(Naming.cameraNamespace(getCamNum()));
            localiseFromAFeatureServer.setPosedEntity(this);
            localiseFromAFeatureServer.setDetectedFeaturesHolder(this);
            nodeMainExecutor.execute(this.localiseFromAFeatureServer, nodeConfiguration8);
        }
        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
//            NodeConfiguration nodeConfiguration8 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
//            nodeConfiguration8.setMasterUri(masterURI);
//            nodeConfiguration8.setNodeName(NODE_NAMESPACE+"localiseFromAFeature_serviceserver_node");
//            this.imagePublisher = new ImagePublisher();
//            imagePublisher.setNodeNamespace(Naming.cameraNamespace(getCamNum()));
//            nodeMainExecutor.execute(this.imagePublisher, nodeConfiguration8);
//  TODO  -  IMAGEPUBLISHER
        }

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


    boolean screenLocked = false;
    boolean registeredAsVisionSource = false;
////    List<DetectedFeature> detectedFeatures = Collections.synchronizedList(new ArrayList<DetectedFeature>());
    // avoid synchronising on the list with e.g.   synchronized (detectedFeatures) { for(detectedFeature: detectedFeatures) {...} }
    // TODO - see http://www.codejava.net/java-core/collections/understanding-collections-and-thread-safety-in-java , https://docs.oracle.com/javase/7/docs/api/java/util/concurrent/CopyOnWriteArrayList.html
    List<DetectedFeature> detectedFeatures = new java.util.concurrent.CopyOnWriteArrayList<DetectedFeature>();



    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        frameNumber++;
        Log.i(TAG,"onCameraFrame: START: cameraNumber="+getCamNum()+": frame="+frameNumber);
        if(!readyToProcessImages) {
            Log.i(TAG,"onCameraFrame: readyToProcessImages is false: returning image without processing.");
            return inputFrame.gray();
        }
        framesProcessed++;

        if (!registeredAsVisionSource & null != registerVisionSourceClient) {
            Log.i(TAG,"onCameraFrame: registering as a vision source.");
            registerVisionSourceClient.registerVisionSource();
            registeredAsVisionSource = true;
        }

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
        System.out.println("MainActivity: onCameraFrame("+matGray.size().width+","+matGray.size().height+"): start");

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
//                System.out.println("MainActivity: onCameraFrame: exception in camera = _cameraBridgeViewBase.camera() : "+e.getMessage());
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
        double BOOFCV_TAG_WIDTH=0.14;
        byte[] current_image_bytes = last_frame_bytes();
        if(null!=current_image_bytes) {
            convertPreview(last_frame_bytes(), camera);
            try {
                FiducialDetector<GrayF32> detector = FactoryFiducial.squareBinary(
                        new ConfigFiducialBinary(BOOFCV_TAG_WIDTH), ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 10), GrayF32.class);  // tag size,  type,  ?'radius'?
                //        detector.setLensDistortion(lensDistortion);

                float  px_pixels = (float)(matGray.size().width/2.0);
                float  py_pixels = (float)(matGray.size().height/2.0);
                double skew = 0.0;
                CameraPinhole pinholeModel = new CameraPinhole(focal_length_in_pixels_x,focal_length_in_pixels_y,skew,px_pixels,py_pixels,new Double(matGray.size().width).intValue(),new Double(matGray.size().height).intValue());
                LensDistortionNarrowFOV pinholeDistort = new LensDistortionPinhole(pinholeModel);
                detector.setLensDistortion(pinholeDistort);  // TODO - do BoofCV calibration - but assume perfect pinhole camera for now
                detector.detect(image);
                Log.i(TAG, "last_frame_bytes: found "+detector.totalFound()+" tags via BoofCV");

                // see https://boofcv.org/index.php?title=Example_Fiducial_Square_Image
                Se3_F64 targetToSensor = new Se3_F64();
                Point2D_F64 locationPixel = new Point2D_F64();
                for (int i = 0; i < detector.totalFound(); i++) {
                    // detector.getImageLocation(i, locationPixel);        // pixel location in input image

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
                    if( detector.hasMessage() )
                        System.out.println("Message   = "+detector.getMessage(i));
                    System.out.println("2D Image Location = "+locationPixel);

                    if( detector.is3D() ) {
                        detector.getFiducialToCamera(i, targetToSensor);

                        Vector3D_F64 transBoofCV_TtoS = targetToSensor.getTranslation();
                        Quaternion_F64 quatBoofCV_TtoS = new Quaternion_F64();
                        ConvertRotation3D_F64.matrixToQuaternion(targetToSensor.getR(),quatBoofCV_TtoS);
                        System.out.println("3D Location: targetToSensor : BoofCV frame : x = "+transBoofCV_TtoS.getX()+", y = "+transBoofCV_TtoS.getY()+", z = "+transBoofCV_TtoS.getZ());
                        System.out.println("3D Location: targetToSensor : BoofCV frame : qx = "+quatBoofCV_TtoS.x+", qy = "+quatBoofCV_TtoS.y+", qz = "+quatBoofCV_TtoS.z+", qw = "+quatBoofCV_TtoS.w);

                        Se3_F64 sensorToTargetIn = null;
                        Se3_F64 sensorToTarget;

                        DenseMatrix64F rotNeg90Y = new DenseMatrix64F( new double[][] { {0,0,-1} , {0,1,0} , {1,0,0} });
                        DenseMatrix64F rotNeg90Z = new DenseMatrix64F( new double[][] { {0,1,0} , {-1,0,0} , {0,0,1} });
                        DenseMatrix64F temp = CommonOps.identity(3) ;
                        DenseMatrix64F new_targetToSensor_r = CommonOps.identity(3) ;
                        CommonOps.mult(rotNeg90Z,rotNeg90Y,temp);
                        CommonOps.mult(targetToSensor.getR(),temp,new_targetToSensor_r);

                        targetToSensor.setRotation(new_targetToSensor_r);

                        sensorToTarget = targetToSensor.invert(sensorToTargetIn);
                        Vector3D_F64 trans_StoT = sensorToTarget.getTranslation();
//                        VisualizeFiducial.drawCube(targetToSensor, param, detector.getWidth(i), 3, g2);
//                        VisualizeFiducial.drawLabelCenter(targetToSensor, param, "" + detector.getId(i), g2);
                        org.ros.rosjava_geometry.Vector3 translation_to_tag_in_robot_convention
                                = new org.ros.rosjava_geometry.Vector3(trans_StoT.getX(), trans_StoT.getY(), trans_StoT.getZ());
//                        org.ros.rosjava_geometry.Vector3 translation_to_tag_in_robot_convention = new org.ros.rosjava_geometry.Vector3(trans.getZ(), trans.getX(), trans.getY());
//                        org.ros.rosjava_geometry.Vector3 translation_to_tag_in_robot_convention = new org.ros.rosjava_geometry.Vector3(trans.getZ(), -trans.getX(), -trans.getY());
                        Quaternion_F64 quat_StoT = new Quaternion_F64();
                        ConvertRotation3D_F64.matrixToQuaternion(sensorToTarget.getR(), quat_StoT);
                        System.out.println("3D Location: sensorToTarget : BoofCV frame : x = "+trans_StoT.getX()+", y = "+trans_StoT.getY()+", z = "+trans_StoT.getZ());
                        System.out.println("3D Location: sensorToTarget : BoofCV frame : qx = "+quat_StoT.x+", qy = "+quat_StoT.y+", qz = "+quat_StoT.z+", qw = "+quat_StoT.w);

                        System.out.println("3D Location: targetToSensor = ");  System.out.println(targetToSensor);  System.out.println(quatBoofCV_TtoS);
                        System.out.println("3D Location: sensorToTarget = ");  System.out.println(sensorToTarget);  System.out.println(quat_StoT);

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
                        detectedFeaturesClient.reportDetectedFeature(tag_id, trans_StoT.getX(), trans_StoT.getY(), trans_StoT.getZ(), quat_StoT.x,quat_StoT.y,quat_StoT.z,quat_StoT.w);
                        if (!poseKnown) {
//                            localiseFromAFeatureClient.localiseFromAFeature(tag_id, trans_StoT.getX(), trans_StoT.getY(), trans_StoT.getZ(), quat_StoT.x,quat_StoT.y,quat_StoT.z,quat_StoT.w);
//                            localiseFromAFeatureClient.localiseFromAFeature(tag_id, trans_StoT.getZ(), -trans_StoT.getX(), -trans_StoT.getY(), quat_StoT.z,-quat_StoT.x,-quat_StoT.y,quat_StoT.w);
//                            localiseFromAFeatureClient.localiseFromAFeature(tag_id, transBoofCV_TtoS.getZ(), -transBoofCV_TtoS.getX(), -transBoofCV_TtoS.getY(), 0,0,0,1);
                            localiseFromAFeatureClient.localiseFromAFeature(tag_id, trans_StoT.getX(), trans_StoT.getY(), trans_StoT.getZ(), quat_StoT.x,quat_StoT.y,quat_StoT.z,quat_StoT.w);
                        }
                    } else {
//                        VisualizeFiducial.drawLabel(locationPixel, "" + detector.getId(i), g2);
                    }
                }

            } catch (Exception e) {
                Log.e(TAG, "onCameraFrame: exception running BoofCV fiducial: ", e);
                e.printStackTrace();
            }

        }

        // end BoofCV


        float  px_pixels = (float)(matGray.size().width/2.0);
        float  py_pixels = (float)(matGray.size().height/2.0);

    if( running_native) {
        String[] tags = aprilTagsUmichOneShot(matGray.getNativeObjAddr(), matRgb.getNativeObjAddr(), tagDetectorPointer, tagSize_metres, focal_length_in_pixels_x, focal_length_in_pixels_y, px_pixels, py_pixels);
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
                    detectedFeaturesClient.reportDetectedFeature(tagId_int, x, y, z, quat.getX(), quat.getY(), quat.getZ(), quat.getW());

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
                System.out.print("MainActivity: onCameraFrame: detectedFeaturesClient is null: cannot report the poses of detected tags");
                System.out.println(tagId);
                System.out.println("-------------------------------------------------------");
            }
        }
    } // end  if( running_native)

        if (screenLocked) {
            System.out.println("onCameraFrame: screenLocked = true at frame "+framesProcessed+": setting output matrices to black");
            Scalar blackScalar = new org.opencv.core.Scalar(0); //,CvType.CV_8UC4
            matRgb.setTo(blackScalar);
        }
        Log.i(TAG,"onCameraFrame: END: cameraNumber="+getCamNum()+": frame="+frameNumber);
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
                System.out.println("MainActivity: onCameraFrame: focal_length_in_pixels_x="+focal_length_in_pixels_x);
                float verticalAngleView = camera.getParameters().getVerticalViewAngle();
                System.out.println("MainActivity: onCameraFrame: getVerticalViewAngle()="+verticalAngleView);
                focal_length_in_pixels_y = (camera.getParameters().getPictureSize().width * 0.5) / tan(verticalAngleView* 0.5 * PI/180.0);
                System.out.println("MainActivity: onCameraFrame: focal_length_in_pixels_y="+focal_length_in_pixels_y);
            } else {
                System.out.println("MainActivity: onCameraFrame: could not get a camera at all : using the Camera 1 API");
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
        System.out.println("MainActivity: onCameraFrame: estimatedFocusedDistances = "+ Arrays.toString(estimatedFocusedDistances));
        MatOfDouble mProjectionCV = new MatOfDouble();
        mProjectionCV.create(3,3, CvType.CV_64FC1);
        final double fovAspectRatio = fieldOfViewX() / fieldOfViewY();
        double diagonalPixels = Math.sqrt( (Math.pow(matGray.size().width, 2.0)) + (Math.pow(matGray.size().width/fovAspectRatio, 2.0)) );
        double diagonalFov = Math.sqrt( (Math.pow(fieldOfViewX(), 2.0)) + (Math.pow(fieldOfViewY(), 2.0)) );
        double focalLengthPixels = diagonalPixels / (2.0 * Math.tan(0.5 * diagonalFov * Math.PI/180.0f ));
        focal_length_in_pixels_x = ( matGray.size().width / (2.0 * Math.tan(0.5 * fieldOfViewX() * Math.PI/180.0f)) );
        focal_length_in_pixels_y = ( matGray.size().height / (2.0 * Math.tan(0.5 * fieldOfViewY() * Math.PI/180.0f)) );
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
    public void setPose(double[] poseXyz, double[] orientationQuaternion_) {
        this.position = poseXyz;
        this.orientation = orientationQuaternion_;
        this.poseKnown = true;
    }

    public void setPose(Pose pose_) {
        setPose(
                new double[]{pose_.getPosition().getX(),pose_.getPosition().getY(),pose_.getPosition().getZ()},
                new double[]{pose_.getOrientation().getX(),pose_.getOrientation().getY(),pose_.getOrientation().getZ(),pose_.getOrientation().getW()});
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

    public native long newTagDetectorKaess();   // Apriltags
    public native void deleteTagDetectorKaess(long tagDetectorPointer);     // Apriltags
    public native String[] aprilTags(long matAddrGray, long matAddrRgb, long tagDetectorPointer, double tagSize_metres, double fx_pixels, double fy_pixels);  // Apriltags

    public native long newTagDetectorUmich();   // Apriltags
    public native void deleteTagDetectorUmich(long tagDetectorPointer, long tagFamilyPointer);     // Apriltags
    public native long aprilTagsUmich(       long matAddrGray, long matAddrRgb, long tagDetectorPointer, long tagFamilyPointer, double tagSize_metres, double fx_pixels, double fy_pixels);  // Apriltags_umich
    public native String[] aprilTagsUmichOneShot(long matAddrGray, long matAddrRgb, long tagDetectorPointer,                        double tagSize_metres, float fx_pixels, float fy_pixels, float px_pixels, float py_pixels);  // Apriltags_umich


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

        FiducialDetector<GrayF32> detector = FactoryFiducial.squareBinary(
                new ConfigFiducialBinary(0.1), ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 10), GrayF32.class);
//        detector.setLensDistortion(lensDistortion);
        detector.detect(image);
        Log.i(TAG, "last_frame_bytes: found "+detector.totalFound()+" tags via BoofCV");
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
    ImageType<boofcv.struct.image.GrayF32> imageType = new ImageType<GrayF32>(GRAY,F32,1);       // TODO - hardcoded
    boofcv.struct.image.GrayF32 image = new boofcv.struct.image.GrayF32();  // TODO - hardcoded
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

