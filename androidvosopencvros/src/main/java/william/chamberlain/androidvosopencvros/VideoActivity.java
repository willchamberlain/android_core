package william.chamberlain.androidvosopencvros;

import android.Manifest;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.pm.PackageManager;
import android.hardware.Camera;
import android.os.Bundle;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.util.Log;

import boofcv.abst.fiducial.FiducialDetector;
import boofcv.struct.image.GrayU8;
import georegression.struct.se.Se3_F64;

import static william.chamberlain.androidvosopencvros.CameraUtils.specs;


/**
 * Detect and locate fiducial markers in the environment by image processing a video stream on an Android device
 * - the actual processing is done by {@link FiducialImageProcessor}
 * which is passed into the super class when {@link #onResume()} is called
 * -  uses BoofCV.
 *
 * Most of the drudgery of
 * video processing is handled by {@link boofcv.android.gui.VideoDisplayActivity}.  This class still needs to tell it which
 * camera to use and needs to select the optimal resolution.  T.
 *
 */
//see https://github.com/lessthanoptimal/BoofCV/blob/SNAPSHOT/integration/boofcv-android/examples/video/app/src/main/java/org/boofcv/video/VideoActivity.java
public class VideoActivity extends boofcv.android.gui.VideoDisplayActivity implements HasCameraIntrinsics, DetectorSource, AlgorithmParameters {
    final Object synchronisationLock = new Object();
    Se3_F64 targetToCamera = new Se3_F64();

    CameraUtils cameraUtils = new CameraUtils();

    /* Algorithm parameters (and state). */
    volatile boolean changed = true;
    volatile boolean robust = true;
    volatile int binaryThreshold = 100;

    /* Camera parameters. */
    boofcv.struct.calib.CameraPinholeRadial cameraIntrinsics;

    /* Application parameters. */
    boolean showInput = true;   // true for showing the (modified) input image, or false for showing debug information only.
    public static ApplicationPreferences preference;


    // see /mnt/nixbig/downloads/boofcv_will/BoofAndroidDemo/app/src/main/java/org/boofcv/android/DemoMain.java
    boolean waitingCameraPermissions = true;
    // see /mnt/nixbig/downloads/boofcv_will/BoofAndroidDemo/app/src/main/java/org/boofcv/android/DemoMain.java
    // If another activity modifies the demo preferences this needs to be set to true so that it knows to reload camera parameters.
    public static boolean changedPreferences = false;


    public void onBinaryThresholdChanged(int binaryThreshold_) {
        synchronized (synchronisationLock) {
            changed = true;
            binaryThreshold = binaryThreshold_;
        }
    }

    public boofcv.struct.calib.CameraPinholeRadial intrinsics() {
        return cameraIntrinsics;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        cameraUtils.loadCameraSpecs(this);
    }


    protected void startDetector() {
        setProcessing(new FiducialImageProcessor(cameraIntrinsics, this, this, this, preference));
    }

    public FiducialDetector<GrayU8> createDetector() {
        return FiducialBinaryUtils.createGrayU8FiducialDetector(binaryThreshold, synchronisationLock, robust);
    }


    @Override
    protected void onResume() {
        super.onResume();

        // SEE /mnt/nixbig/downloads/boofcv_will/BoofAndroidDemo/app/src/main/java/org/boofcv/android/DemoMain.java
        if( !waitingCameraPermissions ) {
            if (preference == null) {
                preference = new ApplicationPreferences();
                setDefaultPreferences();
            } else if (changedPreferences) {
                loadIntrinsic();
            }
        }

        if( !waitingCameraPermissions ) {
            setProcessing(new FiducialImageProcessor<>(cameraIntrinsics, this, this, this, preference));
        }

        // for fun you can display the FPS by uncommenting the line below.
        // The FPS will vary depending on processing time and shutter speed,
        // which is dependent on lighting conditions
//		setShowFPS(true);
    }

    @Override
    protected Camera openConfigureCamera( Camera.CameraInfo cameraInfo )
    {
        Camera mCamera = selectAndOpenCamera(cameraInfo);
        cameraUtils.resizePreviewForCameraResolution(mCamera);

        return mCamera;
    }

    /**
     * Step through the camera list and select a camera.  It is also possible that there is no camera.
     * The camera hardware requirement in AndroidManifest.xml was turned off so that devices with just
     * a front facing camera can be found.  Newer SDK's handle this in a more sane way, but with older devices
     * you need this work around.
     */
    private Camera selectAndOpenCamera(Camera.CameraInfo info) {
        Camera camera = cameraUtils.selectAndOpenCamera(info);
        if (null == camera) {
            dialogNoCamera();
            return null;    // won't ever be called
        } else {
            return camera;
        }
    }

    /**
     * Gracefully handle the situation where a camera could not be found
     */
    private void dialogNoCamera() {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setMessage("Your device has no cameras!")
                .setCancelable(false)
                .setPositiveButton("OK", new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int id) {
                        System.exit(0);
                    }
                });
        AlertDialog alert = builder.create();
        alert.show();
    }

    @Override
    public int binaryThreshold() {
        return binaryThreshold;
    }

    @Override
    public boolean parametersHaveChanged() {
        return changed;
    }

    @Override
    public boolean parametersHaveChanged(boolean newValue) {
        changed = newValue;
        return  parametersHaveChanged();
    }


    // see /mnt/nixbig/downloads/boofcv_will/BoofAndroidDemo/app/src/main/java/org/boofcv/android/DemoMain.java
    private void loadCameraSpecs() {
        int permissionCheck = ContextCompat.checkSelfPermission(this,
                Manifest.permission.CAMERA);

        if( permissionCheck != android.content.pm.PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this,
                    new String[]{Manifest.permission.CAMERA},
                    0);
        } else {
            waitingCameraPermissions = false;
            int numberOfCameras = Camera.getNumberOfCameras();
            for (int i = 0; i < numberOfCameras; i++) {
                CameraSpecs c = new CameraSpecs();
                specs.add(c);

                Camera.getCameraInfo(i, c.info);
                Camera camera = Camera.open(i);
                Camera.Parameters params = camera.getParameters();
                c.horizontalViewAngle = params.getHorizontalViewAngle();
                c.verticalViewAngle = params.getVerticalViewAngle();
                c.sizePreview.addAll(params.getSupportedPreviewSizes());
                c.sizePicture.addAll(params.getSupportedPictureSizes());
                camera.release();
            }
        }
    }

    // see /mnt/nixbig/downloads/boofcv_will/BoofAndroidDemo/app/src/main/java/org/boofcv/android/DemoMain.java
    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           String permissions[], int[] grantResults) {
        switch (requestCode) {
            case 0: {
                // If request is cancelled, the result arrays are empty.
                if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    loadCameraSpecs();
                    preference = new ApplicationPreferences();
                    setDefaultPreferences();
                } else {
                    dialogNoCameraPermission();
                }
                return;
            }
        }
    }

    // see /mnt/nixbig/downloads/boofcv_will/BoofAndroidDemo/app/src/main/java/org/boofcv/android/DemoMain.java
    private void setDefaultPreferences() {
        preference.showFps = false;

        // There are no cameras.  This is possible due to the hardware camera setting being set to false
        // which was a work around a bad design decision where front facing cameras wouldn't be accepted as hardware
        // which is an issue on tablets with only front facing cameras
        if( specs.size() == 0 ) {
            dialogNoCamera();
        }
        // select a front facing camera as the default
        for (int i = 0; i < specs.size(); i++) {
            CameraSpecs c = specs.get(i);

            if( c.info.facing == Camera.CameraInfo.CAMERA_FACING_BACK ) {
                preference.cameraId = i;
                break;
            } else {
                // default to a front facing camera if a back facing one can't be found
                preference.cameraId = i;
            }
        }

        if( !specs.isEmpty() ) {
            CameraSpecs camera = specs.get(preference.cameraId);
            preference.preview = CameraUtils.closest(camera.sizePreview, 320, 240);
            preference.picture = CameraUtils.closest(camera.sizePicture, 640, 480);

            // see if there are any intrinsic parameters to load
            loadIntrinsic();
        }
    }


    // see /mnt/nixbig/downloads/boofcv_will/BoofAndroidDemo/app/src/main/java/org/boofcv/android/DemoMain.java
    private void dialogNoCameraPermission() {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setMessage("Denied access to the camera! Exiting.")
                .setCancelable(false)
                .setPositiveButton("OK", new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int id) {
                        System.exit(0);
                    }
                });
        AlertDialog alert = builder.create();
        alert.show();
    }

//    // see /mnt/nixbig/downloads/boofcv_will/BoofAndroidDemo/app/src/main/java/org/boofcv/android/DemoMain.java
    private void loadIntrinsic() {
        Log.i("VideoActivity","loadIntrinsic(): not even trying to load any INTRINSIC PARAMETERS.");
//        preference.intrinsic = null;
//        try {
//            FileInputStream fos = openFileInput("cam"+preference.cameraId+".txt");
//            Reader reader = new InputStreamReader(fos);
//            preference.intrinsic = CalibrationIO.load(reader);
//        } catch (RuntimeException e) {
//            Log.w("DemoMain", "Failed to load intrinsic parameters: "+e.getClass().getSimpleName());
//            e.printStackTrace();
//        } catch (FileNotFoundException ignore) {
//        }
    }
}