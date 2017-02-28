package william.chamberlain.androidvosopencvros;

import android.support.annotation.NonNull;

/**
 * Created by will on 24/02/17.
 */

public class Naming {

    @NonNull
    public static String cameraNamespace(int camNum) {
        return "cam_" + camNum;
    }


    @NonNull
    public static String cameraFrameId(String suffix_) {
        return "c" + suffix_;
    }

    @NonNull
    public static String cameraFrameId(int camNum) {
        return cameraFrameId(Integer.toString(camNum));
    }

    @NonNull
    public static String cameraBaseFrameId(int camNum) {
        return "b_" + camNum;
    }
}
