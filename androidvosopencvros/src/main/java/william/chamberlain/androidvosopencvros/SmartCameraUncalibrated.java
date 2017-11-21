package william.chamberlain.androidvosopencvros;

import static actionlib_msgs.GoalStatus.ACTIVE;
import static actionlib_msgs.GoalStatus.PENDING;
import static actionlib_msgs.GoalStatus.SUCCEEDED;

/**
 * Created by will on 16/11/17.
 */

// TODO - create this as a node, with the hooks into the perception system - move_base/status - and hooks into the communications -
public class SmartCameraUncalibrated implements SmartCameraBehaviour {
    public void statusChanged(final int robot_id, final byte new_status) {
        // TODO - use the robot_id
        switch (new_status) {
            case PENDING:       // started waiting for planning
                break;
            case ACTIVE:        // started moving
                break;
            case SUCCEEDED:     // got there: now measure position and try estimating a ground plane
                measureRobotPose();
                break;
            default:            // failed in some fashion: go back to previous and try another
        }
    }

    void measureRobotPose() {
        // obtain current robot pose

        // obtain current robot position in image

            // if not visible, set robot goal back to last position

        // if not have enough points to estimate floor homography, set another goal for the robot

        // estimate floor homography

        // if estimate is bad, set robot goal to gather more data

        // if estimate is good, done
    }

}
