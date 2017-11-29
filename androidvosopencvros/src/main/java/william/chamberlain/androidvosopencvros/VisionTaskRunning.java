package william.chamberlain.androidvosopencvros;

import georegression.struct.se.Se3_F64;

/**
 * Created by will on 24/11/17.
 */

public interface VisionTaskRunning {
    void detectedInImage(String robotId_, java.util.Date frameTime_, PixelPosition robotPositionInImage_, Se3_F64 baselink_to_tag_transform_);

    /**
     * Keep track of image frame events - e.g. to track the number of frames
     */
    void imageReceived();
}
