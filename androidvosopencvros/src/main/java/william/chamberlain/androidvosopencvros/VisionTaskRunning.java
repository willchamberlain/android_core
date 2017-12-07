package william.chamberlain.androidvosopencvros;

import org.ros.rosjava_geometry.FrameTransform;

import georegression.struct.se.Se3_F64;

/**
 * Created by will on 24/11/17.
 */

public interface VisionTaskRunning {
    Se3_F64 detectedInImage(String robotId_, java.util.Date imageCaptureTime_, PixelPosition robotPositionInImage_, Se3_F64 baselink_to_tag_transform_, FrameTransform frameTransform);

    /**
     * Keep track of image frame events - e.g. to track the number of frames
     */
    void imageReceived();
}
