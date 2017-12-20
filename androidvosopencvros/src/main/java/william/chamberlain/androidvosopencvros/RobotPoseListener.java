package william.chamberlain.androidvosopencvros;

import org.ros.rosjava_geometry.FrameTransform;

import java.util.List;

import geometry_msgs.PoseWithCovarianceStamped;
import georegression.struct.se.Se3_F64;

/**
 * Created by will on 24/11/17.
 */

public interface RobotPoseListener {
//    void robotPose(String robotId, PoseWithCovarianceStamped pose);
    List<String> robotIds();
    void robotPoseObservation(java.util.Date time_, PixelPosition pixelPosition_, Se3_F64 transformOfFeatureInVisualModel_, FrameTransform transform_);
}
