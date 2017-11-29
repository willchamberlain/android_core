package william.chamberlain.androidvosopencvros;

/**
 * Created by will on 24/11/17.
 */

public interface RobotPoseMonitor {
    void addRobotPoseListener(RobotPoseListener listener_);
    void removeRobotPoseListener(RobotPoseListener listener_);


}
