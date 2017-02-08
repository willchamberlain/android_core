package william.chamberlain.androidvosopencvros;


import android.Manifest;
import android.app.AlertDialog;
import android.content.ActivityNotFoundException;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.content.res.Configuration;
import android.hardware.Camera;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.location.LocationManager;
import android.net.Uri;
import android.os.Bundle;
import android.support.v4.app.ActivityCompat;
import android.view.SurfaceView;
import android.widget.Toast;

import org.opencv.core.CvType;
import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import android.view.Menu;
import android.view.MenuItem;
import android.view.SubMenu;
import android.view.Window;
import android.view.WindowManager;
import android.view.MenuInflater;

import java.io.IOException;
import java.net.URI;


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
        implements CameraBridgeViewBase.CvCameraViewListener2 {

    private static final String TAG = "OCVSample::Activity";
    private CameraBridgeViewBase _cameraBridgeViewBase;

    private long tagDetectorPointer; // Apriltags

    private Mat matGray;
    private Mat matRgb;
    private Mat mRgbaTransposed;
    private Mat mRgbaFlipped;

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

    private LocationManager mLocationManager;
    private SensorManager mSensorManager;


    public MainActivity()
    {
        super("ROS Sensors Driver", "ROS Sensors Driver");
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
            nodeConfiguration.setNodeName("android_sensors_driver_magnetic_field");
            this.magnetic_field_pub = new MagneticFieldPublisher(mSensorManager, sensorDelay);
            nodeMainExecutor.execute(this.magnetic_field_pub, nodeConfiguration);
        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration2 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration2.setMasterUri(masterURI);
            nodeConfiguration2.setNodeName("android_sensors_driver_nav_sat_fix");
            this.fix_pub = new NavSatFixPublisher(mLocationManager);
            nodeMainExecutor.execute(this.fix_pub, nodeConfiguration2);
        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration3 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration3.setMasterUri(masterURI);
            nodeConfiguration3.setNodeName("android_sensors_driver_imu");
            this.imu_pub = new ImuPublisher(mSensorManager, sensorDelay);
            nodeMainExecutor.execute(this.imu_pub, nodeConfiguration3);
        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration4 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration4.setMasterUri(masterURI);
            nodeConfiguration4.setNodeName("android_sensors_driver_pressure");
            this.fluid_pressure_pub = new FluidPressurePublisher(mSensorManager, sensorDelay);
            nodeMainExecutor.execute(this.fluid_pressure_pub, nodeConfiguration4);
        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration5 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration5.setMasterUri(masterURI);
            nodeConfiguration5.setNodeName("android_sensors_driver_illuminance");
            this.illuminance_pub = new IlluminancePublisher(mSensorManager, sensorDelay);
            nodeMainExecutor.execute(this.illuminance_pub, nodeConfiguration5);
        }

        if(currentapiVersion >= android.os.Build.VERSION_CODES.GINGERBREAD){
            NodeConfiguration nodeConfiguration6 = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration6.setMasterUri(masterURI);
            nodeConfiguration6.setNodeName("android_sensors_driver_temperature");
            this.temperature_pub = new TemperaturePublisher(mSensorManager, sensorDelay, tempSensor);
            nodeMainExecutor.execute(this.temperature_pub, nodeConfiguration6);
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
        mRgbaFlipped = new Mat(height,width, CvType.CV_8UC4);
        mRgbaTransposed = new Mat(height,width, CvType.CV_8UC4);
        matRgb = new Mat(height,width, CvType.CV_8UC4);
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
//        Core.flip(matGray,matGray,1);
//        Core.flip(matRgb,matRgb,1);
// TODO - try reducing image size to increase framerate , AND check /Users/will/Downloads/simbaforrest/cv2cg_mini_version_for_apriltag , https://github.com/ikkiChung/MyRealTimeImageProcessing , http://include-memory.blogspot.com.au/2015/02/speeding-up-opencv-javacameraview.html , https://developer.qualcomm.com/software/fastcv-sdk , http://nezarobot.blogspot.com.au/2016/03/android-surfacetexture-camera2-opencv.html , https://www.youtube.com/watch?v=nv4MEliij14 ,
        String[] tags = aprilTags(matGray.getNativeObjAddr(),matRgb.getNativeObjAddr(),tagDetectorPointer);
        for(String tag : tags) {
            System.out.println("-------------------------------------------------------");
            System.out.println(tag);
            System.out.println("-------------------------------------------------------");
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

    public native void salt(long matAddrGray, int nbrElem);
    public native void canny(long matAddrGray);

    public native long newTagDetector();   // Apriltags
    public native void deleteTagDetector(long tagDetectorPointer);     // Apriltags
    public native String[] aprilTags(long matAddrGray, long matAddrRgb, long tagDetectorPointer);  // Apriltags
}

