package william.chamberlain.androidvosopencvros;

import org.ros.rosjava_geometry.FrameTransform;
import org.ros.rosjava_geometry.Transform;

import georegression.struct.se.Se3_F64;

/**
 * Created by will on 17/11/17.
 */

interface RobotPoseMeasure {
    Transform askRobotForPose();
    FrameTransform askRobotForPoseFrame();
//    FrameTransform askRobotForPoseFrameAsync(java.util.Date date);
    void askRobotForPoseFrameAsync(String robotId_, java.util.Date imageFrameTime_, PixelPosition pixelPosition_, Se3_F64 transformOfFeatureInVisualModel_, RobotPoseListener robotPoseListener);
}
