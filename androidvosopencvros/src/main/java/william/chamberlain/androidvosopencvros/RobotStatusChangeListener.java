package william.chamberlain.androidvosopencvros;

import java.util.List;

import actionlib_msgs.GoalStatus;

/**
 * Created by will on 17/11/17.
 */


interface RobotStatusChangeListener {
    void robotStatusChange(java.util.Date statusTime, List<GoalStatus> statuses);
}
