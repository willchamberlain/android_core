package william.chamberlain.androidvosopencvros;

import android.app.AlertDialog;
import android.content.ActivityNotFoundException;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.location.LocationManager;
import android.net.Uri;
import android.os.Bundle;
import android.view.SurfaceView;
import android.widget.Toast;

import org.opencv.core.CvType;
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
import java.util.regex.Matcher;
import java.util.regex.Pattern;


import android.util.Log;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;

/**
 * @author chadrockey@gmail.com (Chad Rockey)
 * @author axelfurlan@gmail.com (Axel Furlan)
 */


public class MainActivity extends RosActivity
        implements CameraBridgeViewBase.CvCameraViewListener2, PosedEntity {


    private static final String TAG = "OCVSample::Activity";
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

    private BaseLoaderCallback _baseLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i(TAG, "OpenCV loaded successfully");
                    // Load ndk built module, as specified in moduleName in build.gradle
                    // after opencv initialization
                    System.loadLibrary("native-lib");
                    _cameraBridgeViewBase.enableView();
                }
                break;
                default: {
                    super.onManagerConnected(status);
                }
            }
        }
    };


    private NavSatFixPublisher fix_pub;
    private ImuPublisher imu_pub;
    private MagneticFieldPublisher magnetic_field_pub;
    private FluidPressurePublisher fluid_pressure_pub;
    private IlluminancePublisher illuminance_pub;
    private TemperaturePublisher temperature_pub;
    private AprilTagsPosePublisher aprilTagsPosePublisher;
    private DetectedFeaturesClient detectedFeaturesClient;
    private MarkerPublisherNode markerPublisherNode;
    private SetPoseServer setPoseServer;

    private LocationManager mLocationManager;
    private SensorManager mSensorManager;


    public MainActivity() {
        super("ROS Sensors Driver", "ROS Sensors Driver");
//        super("VOS AA1", "VOS AA1", URI.create("http://192.168.1.164:11311"));  //TODO // see http://wiki.ros.org/android/Tutorials/indigo/RosActivity , http://answers.ros.org/question/55874/skip-master-chooser/
    }


    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mLocationManager = (LocationManager)this.getSystemService(Context.LOCATION_SERVICE);
        mSensorManager = (SensorManager)this.getSystemService(SENSOR_SERVICE);


        // Load ndk built module, as specified
        // in moduleName in build.gradle
        System.loadLibrary("native-lib");
        System.loadLibrary("apriltags_kaess");
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_main);

//        // Permissions for Android 6+
//        ActivityCompat.requestPermissions(MainActivity.this,
//                new String[]{Manifest.permission.CAMERA},
//                1);

        _cameraBridgeViewBase = (CameraBridgeViewBase) findViewById(R.id.main_surface);
        System.out.println("MainActivity: onCreate: before running _cameraBridgeViewBase.setMaxFrameSize(640,480)");
        _cameraBridgeViewBase.setMaxFrameSize(640,480);  // http://stackoverflow.com/questions/17868954/android-opencv-how-to-set-camera-resolution-when-using-camerabridgeviewbase
        System.out.println("MainActivity: onCreate: after running _cameraBridgeViewBase.setMaxFrameSize(640,480)");
        _cameraBridgeViewBase.setVisibility(SurfaceView.VISIBLE);
        _cameraBridgeViewBase.setCvCameraViewListener(this);

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

//        System.out.println("MainActivity: onResume: before running AndroidCameraAdapterForDepricatedApi.setCameraToLowestResolution()");
//        AndroidCameraAdapterForDepricatedApi.setCameraToLowestResolution();
//        System.out.println("MainActivity: onResume: after running AndroidCameraAdapterForDepricatedApi.setCameraToLowestResolution()");

        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, _baseLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            _baseLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }


    @Override
    protected void init(NodeMainExecutor nodeMainExecutor)
    {
        URI masterURI = getMasterUri();
        //masterURI = URI.create("http://localhost:11311/");
        //masterURI = URI.create("http://192.168.15.247:11311/");
        //masterURI = URI.create("http://10.0.1.157:11311/");
        System.out.print("init: masterURI=");System.out.println(masterURI);
        final String NODE_NAMESPACE = Naming.cameraNamespace(getCamNum());
        System.out.print("init: camNum=");System.out.print(getCamNum()); System.out.print("NODE_NAMESPACE = ");System.out.println(NODE_NAMESPACE);

        int currentapiVersion = android.os.Build.VERSION.SDK_INT;

        int sensorDelay = 20000; // 20,000 us == 50 Hz for Android 3.1 and above
        if(currentapiVersion <= android.os.Build.VERSION_CODES.HONEYCOMB){
            sensorDelay = SensorManager.SENSOR_DELAY_UI; // 16.7Hz for older devices.  They only support enum values, not the microsecond version.
        }

        @SuppressWarnings("deprecation")
        int tempSensor = Sensor.TYPE_TEMPERATURE; // Older temperature
        if(currentapiVersion <= android.os.Build.VERSION_CODES.ICE_CREAM_SANDWICH){
            tempSensor = Sensor.TYPE_AMBIENT_TEMPERATURE; // Use newer temperature if possible
        }


        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration.setMasterUri(masterURI);
            nodeConfiguration.setNodeName(NODE_NAMESPACE+"android_sensors_driver_magnetic_field");
            this.magnetic_field_pub = new MagneticFieldPublisher(mSensorManager, sensorDelay);
            nodeMainExecutor.execute(this.magnetic_field_pub, nodeConfiguration);
        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration2 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration2.setMasterUri(masterURI);
            nodeConfiguration2.setNodeName(NODE_NAMESPACE+"android_sensors_driver_nav_sat_fix");
            this.fix_pub = new NavSatFixPublisher(mLocationManager);
            nodeMainExecutor.execute(this.fix_pub, nodeConfiguration2);
        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration3 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration3.setMasterUri(masterURI);
            nodeConfiguration3.setNodeName(NODE_NAMESPACE+"android_sensors_driver_imu");
            this.imu_pub = new ImuPublisher(mSensorManager, sensorDelay);
            nodeMainExecutor.execute(this.imu_pub, nodeConfiguration3);
        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration4 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration4.setMasterUri(masterURI);
            nodeConfiguration4.setNodeName(NODE_NAMESPACE+"android_sensors_driver_pressure");
            this.fluid_pressure_pub = new FluidPressurePublisher(mSensorManager, sensorDelay);
            nodeMainExecutor.execute(this.fluid_pressure_pub, nodeConfiguration4);
        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration5 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration5.setMasterUri(masterURI);
            nodeConfiguration5.setNodeName(NODE_NAMESPACE+"android_sensors_driver_illuminance");
            this.illuminance_pub = new IlluminancePublisher(mSensorManager, sensorDelay);
            nodeMainExecutor.execute(this.illuminance_pub, nodeConfiguration5);
        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration6 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration6.setMasterUri(masterURI);
            nodeConfiguration6.setNodeName(NODE_NAMESPACE+"android_sensors_driver_temperature");
            this.temperature_pub = new TemperaturePublisher(mSensorManager, sensorDelay, tempSensor);
            nodeMainExecutor.execute(this.temperature_pub, nodeConfiguration6);
        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration7 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration7.setMasterUri(masterURI);
            nodeConfiguration7.setNodeName(NODE_NAMESPACE+"apriltags_pose_publisher");
            this.aprilTagsPosePublisher = new AprilTagsPosePublisher();
            nodeMainExecutor.execute(this.aprilTagsPosePublisher, nodeConfiguration7);
        }

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
            nodeConfiguration.setNodeName(NODE_NAMESPACE+"apriltags_marker_publisher");
            this.markerPublisherNode = new MarkerPublisherNode();
            markerPublisherNode.setNodeNamespace(NODE_NAMESPACE);
            nodeMainExecutor.execute(this.markerPublisherNode, nodeConfiguration);
        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration.setMasterUri(masterURI);
            nodeConfiguration.setNodeName(NODE_NAMESPACE+"apriltags_marker_publisher");
            this.setPoseServer = new SetPoseServer();
            setPoseServer.setNodeNamespace(NODE_NAMESPACE);
            setPoseServer.setPosedEntity(this);
            nodeMainExecutor.execute(this.setPoseServer, nodeConfiguration);
        }

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
        deleteTagDetector(tagDetectorPointer); // Apriltags
        disableCamera();
    }

    public void disableCamera() {
        if (_cameraBridgeViewBase != null)
            _cameraBridgeViewBase.disableView();
    }

    public void onCameraViewStarted(int width, int height) {
        System.out.println("MainActivity: onCameraViewStarted("+width+","+height+"): start");
        mRgbaFlipped = new Mat(height,width, CvType.CV_8UC4);
        mRgbaTransposed = new Mat(height,width, CvType.CV_8UC4);
        matRgb = new Mat(height,width, CvType.CV_8UC4);
        System.out.println("MainActivity: onCameraViewStarted("+width+","+height+"): end");
    }

    public void onCameraViewStopped() {
        if( null != matGray) { matGray.release(); }
        if( null != matRgb) { matRgb.release(); }
        if( null != mRgbaFlipped) { mRgbaFlipped.release(); }
        if( null != mRgbaTransposed) { mRgbaTransposed.release(); }
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
//        Display display = ((WindowManager) this.getSystemService(Context.WINDOW_SERVICE)).getDefaultDisplay();
//        int rotation = display.getRotation();
//        Log.d(TAG, "rotation = " + rotation);

        matGray = inputFrame.gray();
        matRgb  = inputFrame.rgba();
        System.out.println("MainActivity: onCameraFrame("+matGray.size().width+","+matGray.size().height+"): start");
//        Core.flip(matGray,matGray,1);
//        Core.flip(matRgb,matRgb,1);
// TODO - try reducing image size to increase framerate , AND check /Users/will/Downloads/simbaforrest/cv2cg_mini_version_for_apriltag , https://github.com/ikkiChung/MyRealTimeImageProcessing , http://include-memory.blogspot.com.au/2015/02/speeding-up-opencv-javacameraview.html , https://developer.qualcomm.com/software/fastcv-sdk , http://nezarobot.blogspot.com.au/2016/03/android-surfacetexture-camera2-opencv.html , https://www.youtube.com/watch?v=nv4MEliij14 ,
        String[] tags = aprilTags(matGray.getNativeObjAddr(),matRgb.getNativeObjAddr(),tagDetectorPointer);
        for(String tag : tags) {
            System.out.println("-------------------------------------------------------");
            System.out.print("---");
            System.out.print(tag);
            System.out.println("---");
        }
        System.out.println("--------------------------------------------------------------------------------------------------------------");

        Time timeNow = Date.nowAsTime();
        for(String tag : tags) {
            {
                System.out.println("-------------------------------------------------------");
                System.out.print("---"); System.out.print(tag); System.out.println("---");
                System.out.println("   checking for pattern  [[" + tagPattern.toString() + "]]");
                System.out.println("   ... in string [[" + tag + "]]");
            }
            Matcher matcher = tagPattern.matcher(tag);
            {
                System.out.print("--- matcher matches ? "); System.out.println(matcher.matches());
            }
            String tagId = matcher.group(1);
            Integer tagId_integer = Integer.parseInt(tagId);
            int tagId_int = tagId_integer.intValue();
            {
                System.out.print("--- tag_id="); System.out.print(tagId); System.out.print(" x=");  System.out.print(matcher.group(2)); System.out.println("---");
                System.out.println("-------------------------------------------------------");
            }
            if(null != aprilTagsPosePublisher) {aprilTagsPosePublisher.publishAprilTagId(Integer.parseInt(tagId));}
            else { System.out.print("MainActivity: onCameraFrame: aprilTagsPosePublisher is null: cannot publish tag id "); System.out.println(tagId); }

            if(null != detectedFeaturesClient) {
// ("tag ([0-9]+) at x=([0-9-]+\\.[0-9]+) y=([0-9-]+\\.[0-9]+) z=([0-9-]+\\.[0-9]+) roll=([0-9-]+\\.[0-9]+) pitch=([0-9-]+\\.[0-9]+) yaw=([0-9-]+\\.[0-9]+) qx=([0-9-]+\\.[0-9]+) qy=([0-9-]+\\.[0-9]+) qz=([0-9-]+\\.[0-9]+) qw=([0-9-]+\\.[0-9]+)");
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
                detectedFeaturesClient.reportDetectedFeature(tagId_int, x,y,z,qx,qy,qz,qw);
//                System.out.println("--- detectedFeaturesClient.reportDetectedFeature --- ");
//                System.out.print("--- tag_id=");System.out.print(tagId);
//                    System.out.print(" :  x=");System.out.print(matcher.group(2));System.out.print(" y=");System.out.print(matcher.group(3));System.out.print(" z=");System.out.print(matcher.group(4));
//                    System.out.print(" :  roll=");System.out.print(matcher.group(5));System.out.print(" pitch=");System.out.print(matcher.group(6));System.out.print(" yaw=");System.out.print(matcher.group(7));
//                    System.out.print(" :  qx=");System.out.print(matcher.group(8));System.out.print(" qy=");System.out.print(matcher.group(9));System.out.print(" qz=");System.out.print(matcher.group(10));System.out.print(" qw=");System.out.print(matcher.group(11));
//                    System.out.println("---");
//                System.out.println("-------------------------------------------------------");
                {
                    //markerPublisherNode.publishMarker(String marker_namespace_, int marker_id_, String marker_text_, double x,double y,double z,double qx,double qy,double qz,double qw, String parent_frame_id, Time time_) {
                    markerPublisherNode.publishMarker(MARKER_NAMESPACE, tagId_int, tagId, x, y, z, qx, qy, qz, qw, Naming.cameraFrameId(getCamNum()), timeNow);
                    // TODO - use same variable for this and aa1_vos_android_catkin_ws___src/vos_aa1/src/vos_aa1/detect_feature_server.py
                    // TODO - publish markers in detected_feature_server.py - once I have figured out what is going on with the RPY in that Python code
                    // TODO -   ... or use C++ as detected_feature_server.cpp
                }
            }
            else {
                System.out.print("MainActivity: onCameraFrame: detectedFeaturesClient is null: cannot report the poses of detected tags"); System.out.println(tagId);
                System.out.println("-------------------------------------------------------");
            }
        }


//        mRgbaTransposed = matRgb.t();
//        Imgproc.resize(mRgbaTransposed, mRgbaFlipped, matRgb.size(),0,0,0);
//        Core.flip(mRgbaFlipped, matRgb, 1); // see - http://answers.opencv.org/question/20325/how-can-i-change-orientation-without-ruin-camera-settings/

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


    @Override
    public void setPose(double[] poseXyz, double[] orientationQuaternion_) {
        this.position = poseXyz;
        this.orientation = orientationQuaternion_;
    }

    public double[] getPosition() {
        return position;
    }

    public double[] getOrientation() {
        return orientation;
    }


    public native void salt(long matAddrGray, int nbrElem);
    public native void canny(long matAddrGray);

    public native long newTagDetector();   // Apriltags
    public native void deleteTagDetector(long tagDetectorPointer);     // Apriltags
    public native String[] aprilTags(long matAddrGray, long matAddrRgb, long tagDetectorPointer);  // Apriltags



}

