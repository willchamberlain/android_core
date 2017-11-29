package william.chamberlain.androidvosopencvros;

import java.util.List;

import geometry_msgs.PoseWithCovarianceStamped;

/**
 * Created by will on 24/11/17.
 */

public interface RobotPoseListener {
    void robotPose(String robotId, PoseWithCovarianceStamped pose);
    List<String> robotIds();
}
