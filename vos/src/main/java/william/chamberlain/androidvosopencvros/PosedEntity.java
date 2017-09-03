package william.chamberlain.androidvosopencvros;

import geometry_msgs.Pose;

/**
 * Created by will on 28/02/17.
 */

public interface PosedEntity {
    void setPose(double[] poseXyz, double[] orientationQuaternionXyzw_);
    void setPose(Pose pose);
    double[] getPositionXyz();
    double[] getOrientationQuaternionXyzw();
    boolean poseKnown();
}
