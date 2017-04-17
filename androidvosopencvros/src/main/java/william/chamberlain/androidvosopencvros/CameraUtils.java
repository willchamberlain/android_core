package william.chamberlain.androidvosopencvros;

import android.Manifest;
import android.app.Activity;
import android.hardware.Camera;
import android.support.annotation.Nullable;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;

import java.util.ArrayList;
import java.util.List;

import boofcv.struct.calib.CameraPinholeRadial;
import georegression.metric.UtilAngle;

/**
 * Created by will on 12/04/17.
 */

public class CameraUtils {
    boolean waitingCameraPermissions = true;

    // contains information on all the cameras.  less error prone and easier to deal with
    public static List<CameraSpecs> specs = new ArrayList<CameraSpecs>();

    /**
     * Either loads the current cameraIntrinsics parameters or makes one up from camera information
     * if it doesn't exist.
     *
     * From org.boofcv.android.misc.MiscUtils
     */
    public static CameraPinholeRadial checkThenInventIntrinsic(HasCameraIntrinsics hasCameraIntrinsics) {

        CameraPinholeRadial intrinsic;

        // make sure the camera is calibrated first
        if( hasCameraIntrinsics.intrinsics() == null ) {
            CameraSpecs specs = CameraUtils.specs.get(DemoVideoDisplayActivity.preference.cameraId);

            Camera.Size size = specs.sizePreview.get( DemoVideoDisplayActivity.preference.preview);

            intrinsic = new CameraPinholeRadial();

            double hfov = UtilAngle.degreeToRadian(specs.horizontalViewAngle);
            double vfov = UtilAngle.degreeToRadian(specs.verticalViewAngle);

            intrinsic.width = size.width; intrinsic.height = size.height;
            intrinsic.cx = intrinsic.width/2;
            intrinsic.cy = intrinsic.height/2;
            intrinsic.fx = intrinsic.cx / Math.tan(hfov/2.0f);
            intrinsic.fy = intrinsic.cy / Math.tan(vfov/2.0f);
        } else {
            intrinsic = hasCameraIntrinsics.intrinsics();
        }

        return intrinsic;
    }

    /**
     * Goes through the size list and selects the one which is the closest specified size
     */
    public static int closest( List<Camera.Size> sizes , int width , int height ) {
        int best = -1;
        int bestScore = Integer.MAX_VALUE;

        for( int i = 0; i < sizes.size(); i++ ) {
            Camera.Size s = sizes.get(i);

            int dx = s.width-width;
            int dy = s.height-height;

            int score = dx*dx + dy*dy;
            if( score < bestScore ) {
                best = i;
                bestScore = score;
            }
        }

        return best;
    }

    public void loadCameraSpecs(Activity activityForPermissions) {
        int permissionCheck = ContextCompat.checkSelfPermission(activityForPermissions,
                Manifest.permission.CAMERA);

        if( permissionCheck != android.content.pm.PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(activityForPermissions,
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

    @Nullable
    Camera selectAndOpenCamera(Camera.CameraInfo info) {
        int numberOfCameras = Camera.getNumberOfCameras();

        int selected = -1;

        for (int i = 0; i < numberOfCameras; i++) {
            Camera.getCameraInfo(i, info);

            if( info.facing == Camera.CameraInfo.CAMERA_FACING_BACK ) {
                selected = i;
                break;
            } else {
                // default to a front facing camera if a back facing one can't be found
                selected = i;
            }
        }

        if( selected == -1 ) {
            return null;
        } else {
            return Camera.open(selected);
        }
    }

    public void resizePreviewForCameraResolution(Camera mCamera) {
        Camera.Parameters param = mCamera.getParameters();
        // Select the preview size closest to 320x240: Smaller images are recommended because some computer vision operations are very expensive
        List<Camera.Size> sizes = param.getSupportedPreviewSizes();
        Camera.Size s = sizes.get(closest(sizes,320,240));
        param.setPreviewSize(s.width,s.height);
        mCamera.setParameters(param);
    }
}
