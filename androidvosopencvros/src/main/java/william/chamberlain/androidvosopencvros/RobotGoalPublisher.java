package william.chamberlain.androidvosopencvros;

import org.ros.rosjava_geometry.Transform;

import geometry_msgs.PoseStamped;

/**
 * Created by will on 20/11/17.
 */

interface RobotGoalPublisher {
    void sendRobotGoalInWorldFrame(PoseStamped poseStamped_);

    void sendRobotGoalInWorldFrame(Transform transform_);

    void sendRobotGoalInRobotFrame(Transform transform_);
}
