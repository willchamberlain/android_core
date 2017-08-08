package william.chamberlain.androidvosopencvros;

import georegression.struct.point.Point2D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Quaternion_F64;

/**
 * Data type for target detections from a sensor.
 */

public class DetectedTag {
    private int tag_id;
    private Se3_F64 sensorToTargetViaTransform;
    private Quaternion_F64 sensorToTargetViaTransformQuat;
    private Point2D_F64 locationPixel;

    public DetectedTag(final int tag_id_, final Se3_F64 sensorToTargetViaTransform_, final Quaternion_F64 sensorToTargetViaTransformQuat_) {
        tag_id = tag_id_;
        sensorToTargetViaTransform = sensorToTargetViaTransform_;
        sensorToTargetViaTransformQuat = sensorToTargetViaTransformQuat_;
    }

    public DetectedTag(final int tag_id_, final Se3_F64 sensorToTargetViaTransform_, final Point2D_F64 locationPixel_) {
        tag_id = tag_id_;
        sensorToTargetViaTransform = sensorToTargetViaTransform_;
        locationPixel = locationPixel_;
    }



    public int getTag_id() {
        return tag_id;
    }

    public Se3_F64 getSensorToTargetViaTransform() {
        return sensorToTargetViaTransform;
    }

    public Quaternion_F64 getSensorToTargetViaTransformQuat() {
        return sensorToTargetViaTransformQuat;
    }

    public Point2D_F64 getLocationPixel() {
        return locationPixel;
    }
}
