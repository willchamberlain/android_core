package william.chamberlain.androidvosopencvros;

import org.ros.rosjava_geometry.Transform;

/**
 * Created by will on 17/11/17.
 */

interface RobotPoseMeasure {
    Transform askRobotForPose();
}
