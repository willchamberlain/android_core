package william.chamberlain.androidvosopencvros;

import actionlib_msgs.GoalStatus;

/**
 * Created by will on 17/11/17.
 */


interface RobotStatusMonitor {
    GoalStatus[] robotStatusChange();
    void addRobotStatusChangeListener(RobotStatusChangeListener listener_);
    void removeRobotStatusChangeListener(RobotStatusChangeListener listener_);
}
