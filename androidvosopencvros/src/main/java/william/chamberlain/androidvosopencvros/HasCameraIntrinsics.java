package william.chamberlain.androidvosopencvros;

/**
 * Created by will on 12/04/17.
 */

public interface HasCameraIntrinsics {
    boofcv.struct.calib.CameraPinholeRadial intrinsics();
}
