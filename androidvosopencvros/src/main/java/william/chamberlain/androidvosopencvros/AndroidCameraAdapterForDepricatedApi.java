package william.chamberlain.androidvosopencvros;

import android.content.Context;
import android.content.pm.PackageManager;
import android.hardware.Camera;

import java.util.List;

/**
 * Handler for the Android camera
 * not using Intents which delegate to default implementations in apps and components (see https://developer.android.com/reference/android/content/Intent.html)
 * but uses the depricated API - see https://developer.android.com/guide/topics/media/camera.html
 * Created by will on 14/02/17.
 */

public class AndroidCameraAdapterForDepricatedApi {


    /** Check if this device has a camera */
    public static boolean checkCameraHardware(Context context) {
        if (context.getPackageManager().hasSystemFeature(PackageManager.FEATURE_CAMERA)){
            // this device has a camera
            return true;
        } else {
            // no camera on this device
            return false;
        }
    }

    /** Gets an instance of the Camera object safely: returns null if there is no available camera. */
    public static Camera getCameraInstance(){
        Camera camera = null;
        try {
            camera = Camera.open(); // attempt to get a Camera instance
        }
        catch (Exception e){
            // Camera is not available (in use or does not exist)
        }
        return camera; // returns null if camera is unavailable
    }


    /** Sets resolution safely: returns null if there is no available camera. */
    public static Camera setCameraToLowestResolution(){
        Camera camera = getCameraInstance();
        if (null != camera) {
            Camera.Parameters cameraParameters = camera.getParameters();
            cameraParameters = setCameraParametersResolutionLow(cameraParameters);
            camera.setParameters(cameraParameters);
        }
        return camera; // returns null if camera is unavailable
    }

    /** Sets resolution safely: returns null if there is no available camera. */
    public static Camera setCameraToHighestResolution(){
        Camera camera = getCameraInstance();
        if (null != camera) {
            Camera.Parameters cameraParameters = camera.getParameters();
            cameraParameters = setCameraParametersResolutionHigh(cameraParameters);
            camera.setParameters(cameraParameters);
        }
        return camera; // returns null if camera is unavailable
    }

    /**
     * Sets the resolution in the Camera.Parameters to the lowest available in those Camera.Parameters.
     * @param cameraParameters
     * @return the Camera.Parameters with the resolution set to the lowest available in cameraParameters.
     */
    public static Camera.Parameters setCameraParametersResolutionLow(Camera.Parameters cameraParameters) {
        List<Camera.Size> pictureSizes = new CameraResolutionComparator(cameraParameters).sortByWidthAndHeight();
        cameraParameters.setPictureSize(pictureSizes.get(0).width,pictureSizes.get(0).height);
        return cameraParameters;
    }


    /**
     * Sets the resolution in the Camera.Parameters to the highest available in those Camera.Parameters.
     * @param cameraParameters
     * @return the Camera.Parameters with the resolution set to the highest available in cameraParameters.
     */
    public static Camera.Parameters setCameraParametersResolutionHigh(Camera.Parameters cameraParameters) {
        List<Camera.Size> pictureSizes = new CameraResolutionComparator(cameraParameters).sortByWidthAndHeight();
        cameraParameters.setPictureSize(pictureSizes.get(pictureSizes.size()-1).width,pictureSizes.get(pictureSizes.size()-1).height);
        return cameraParameters;
    }


}
