package william.chamberlain.androidvosopencvros;

import android.hardware.Camera;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Created by will on 14/02/17.
 */
class CameraResolutionComparator {
    private Camera.Parameters cameraParameters;

    public CameraResolutionComparator(Camera.Parameters cameraParameters) {
        this.cameraParameters = cameraParameters;
    }

    public List<Camera.Size> sortByWidthAndHeight() {
        List<Camera.Size> pictureSizes = cameraParameters.getSupportedPictureSizes();
        Collections.sort(pictureSizes, new Comparator<Camera.Size>() {
            @Override
            public int compare(Camera.Size size1, Camera.Size size2) {
                if (size1.width == size2.width) {
                    return size1.height - size2.height;
                } else {
                    return size1.width - size2.width;
                }
            }
        });
        return pictureSizes;
    }
}
