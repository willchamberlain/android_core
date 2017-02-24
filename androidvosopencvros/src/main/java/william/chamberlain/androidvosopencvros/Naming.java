package william.chamberlain.androidvosopencvros;

import android.support.annotation.NonNull;

/**
 * Created by will on 24/02/17.
 */

public class Naming {
    @NonNull
    public static String cameraFrameId(String suffix_) {
        return "c_" + suffix_;
    }

    @NonNull
    public static String cameraFrameId(int camNum) {
        return "c_" + cameraNamespace(camNum);
    }

    @NonNull
    public static String cameraNamespace(int camNum) {
        return "cam_" + camNum + "_";
    }
}
