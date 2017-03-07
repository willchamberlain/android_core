package william.chamberlain.androidvosopencvros;

import geometry_msgs.Pose;

/**
 * Created by will on 28/02/17.
 */

public interface PosedEntity {
    void setPose(double[] poseXyz, double[] orientationQuaternion_);
    void setPose(Pose pose);
    double[] getPosition();
    double[] getOrientation();
}
