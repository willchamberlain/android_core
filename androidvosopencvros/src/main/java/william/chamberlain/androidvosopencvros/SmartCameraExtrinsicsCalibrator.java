package william.chamberlain.androidvosopencvros;

import android.support.annotation.NonNull;

import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;
import org.ros.rosjava_geometry.Quaternion;

import java.util.ArrayList;
import java.util.List;

import actionlib_msgs.GoalStatus;
import william.chamberlain.androidvosopencvros.ros_types.RosTypes;

import static actionlib_msgs.GoalStatus.ACTIVE;
import static actionlib_msgs.GoalStatus.PENDING;
import static actionlib_msgs.GoalStatus.SUCCEEDED;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.SelfState.calibrated;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.SelfState.recordObs;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.SelfState.robotMoving;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.SelfState.timeout_waitingForObs;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.SelfState.uncalibrated;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.SelfState.waitingForObs;

/**
 * Created by will on 17/11/17.
 */

public class SmartCameraExtrinsicsCalibrator implements RobotStatusChangeListener {

    class Observation {
        PixelPosition pixelPosition;
        Transform pose;
        public Observation(PixelPosition pixelPosition_, Transform pose_) {
            this.pixelPosition=pixelPosition_;
            this.pose=pose_;
        }
    }


    enum SelfState {
        uncalibrated, seenRobot, robotMoving, waitingForObs, timeout_waitingForObs, recordObs, calibrated;
    }
//    enum RobotState {
//        unknown,robotMoving, robotStopped;
//    }

    List<Observation> observations = new ArrayList<>(4);
    List<Transform> robotGoalPoses = new ArrayList<>(1);
    SelfState state = uncalibrated;
//    RobotState robotState = unknown;


    RobotGoalPublisher robotGoalPublisher;

    int waitCount = 0;

    // frame tag
    public void robotDetectedInImage(PixelPosition robotPositionInImage) {
        System.out.println("robotDetectedInImage: start: state == "+state+", robotPositionInImage="+robotPositionInImage);
        if(state == uncalibrated || state == waitingForObs) {
            state = recordObs;
            Transform robotPose = askRobotForItsPose();
            System.out.println("robotDetectedInImage: robotPose = "+robotPose);
            observations.add(new Observation(robotPositionInImage, robotPose));
            if(planRobotPositions()) {
                askRobotToMove();
            } else {
                System.out.println("robotDetectedInImage: PLANNING FAILED: state == "+state+", robotPositionInImage="+robotPositionInImage);
            }
            state = robotMoving;
        }
        System.out.println("robotDetectedInImage: end: state == "+state+", robotPositionInImage="+robotPositionInImage);
    }

    int robotFinishedMoving_waitCount = 0;

    final static int FPS_EST         = 5;
    final static int SECONDS_TO_WAIT = 5;

    public void finishedWithImage() {       //  MainActivity onCameraFrame is basically the timing driver at the moment
        if(state == waitingForObs) {
            if(robotFinishedMoving_waitCount < FPS_EST*SECONDS_TO_WAIT) {
                System.out.println("finishedWithImage: robotFinishedMoving_waitCount="+robotFinishedMoving_waitCount+" : waiting ...");
                robotFinishedMoving_waitCount++;
            } else {
                System.out.println("finishedWithImage: robotFinishedMoving_waitCount="+robotFinishedMoving_waitCount+" : waited long enough : "+stateString());
                state = timeout_waitingForObs;
                if (planRobotPositions() ) {
                    askRobotToMove();
                    System.out.println("finishedWithImage: PLANNING FAILED: robotFinishedMoving_waitCount=" + robotFinishedMoving_waitCount + " : " + stateString());
                } else {
                    robotFinishedMoving_waitCount = 0;
                    System.out.println("finishedWithImage: no observations - waiting some more - robotFinishedMoving_waitCount=" + robotFinishedMoving_waitCount + " : " + stateString());
                }
            }
        }
    }



    private RobotPoseMeasure robotPoseMeasure;

    public void setRobotPoseMeasure(RobotPoseMeasure robotPoseMeasure_) {
        this.robotPoseMeasure = robotPoseMeasure_;
    }

    protected Transform askRobotForItsPose() {
        System.out.println("askRobotForItsPose: "+stateString());
        Transform transform = robotPoseMeasure.askRobotForPose();
        System.out.println("askRobotForItsPose: transform ="+transform);
        return transform;
    }



    protected void estimateExtrinsics() {
        for(Observation observation : observations) {
        }
    }

    protected boolean planRobotPositions() {
        if(null != robotGoalPoses && robotGoalPoses.size() > 0) {
            System.out.println("planRobotPositions: already have goal poses: "+stateString());
            return true;
        } else {
            if (null==observations || observations.size()<=0) {
                System.out.println("planRobotPositions: no observations: cannot calculate pose: "+stateString());
                return false;
            }
            System.out.println("planRobotPositions: calculating poses: "+stateString());
//            List<Transform> plannedRobotPositions = new ArrayList<>();
            float scale = 1.5f;
            Observation lastObservation = observations.get(observations.size() - 1);
            Quaternion robotGoalRotation = RosTypes.copyQuaternion(lastObservation.pose);
            Vector3 robotGoalPosition = RosTypes.copyVector3(lastObservation.pose);
            robotGoalPoses = new ArrayList<Transform>(1);

//            for (int x = -1; x <= 1; x++) {
//                for (int y = -1; y <= 1; y++) {
//                    Vector3 changeFromFirstObservation = new Vector3((double) x * scale, (double) y * scale, 0);
//                    robotGoalPosition = robotGoalPosition.add(changeFromFirstObservation);
//                    Transform robotGoalPose = new Transform(robotGoalPosition, robotGoalRotation);
//                    robotGoalPoses.add(robotGoalPose);
//                }
//            }

            Vector3 fixedPosition = new Vector3(1.7, -3.25, 0);
            Transform robotGoalPose = new Transform(fixedPosition, robotGoalRotation);
            robotGoalPoses.add(robotGoalPose);
            fixedPosition = new Vector3(2.09, -2.25, 0);
            robotGoalPose = new Transform(fixedPosition, robotGoalRotation);
            robotGoalPoses.add(robotGoalPose);

            return true;
        }
    }

    Transform nextPose;

    protected void askRobotToMove() {
        System.out.println("askRobotToMove: start: "+stateString());
        state = robotMoving;
        nextPose = robotGoalPoses.remove(0);
        robotGoalPublisher.sendRobotGoal(nextPose);
        System.out.println("askRobotToMove: end: "+stateString());
    }

    protected void robotIsMoving() {
        System.out.println("robotIsMoving: "+stateString());
    }

    protected void robotFinishedMoving() {
        System.out.println("robotFinishedMoving: start: "+stateString());
        if(state == robotMoving) {
            state      = waitingForObs;
            robotFinishedMoving_waitCount = 0;
        }
        System.out.println("robotFinishedMoving: end: "+stateString());
    }

    @Override
    public void robotStatusChange(List<GoalStatus> statuses) {
        System.out.println("robotStatusChange: start: "+stateString());
        if(null != statuses && 0 < statuses.size()) {
            System.out.println("robotStatusChange: statuses.size()=="+statuses.size());
            GoalStatus status = statuses.get(0);
            final byte status_now = status.getStatus();
            switch (status_now) {
                case PENDING:       // started waiting for planning
                    System.out.println("robotStatusChange: PENDING");
                    robotIsMoving();
                    break;
                case ACTIVE:        // started moving
                    System.out.println("robotStatusChange: ACTIVE");
                    robotIsMoving();
                    break;
                case SUCCEEDED:     // got there: now measure position and try estimating a ground plane
                    System.out.println("robotStatusChange: SUCCEEDED");
                    robotFinishedMoving();
                    break;
                default:            // failed in some fashion: go back to previous and try another
                    System.out.println("robotStatusChange: "+status_now);
                    robotFinishedMoving();
            }
        } else {

        }
        System.out.println("robotStatusChange: end: "+stateString());
    }



    /*****************************************************************/

    public void setRobotGoalPublisher(RobotGoalPublisher robotGoalPublisher) {
        this.robotGoalPublisher = robotGoalPublisher;
    }


    public String stateString() {
        return "state=" + state + ", robotFinishedMoving_waitCount="+robotFinishedMoving_waitCount;
    }


    /*****************************************************************/

    public SmartCameraExtrinsicsCalibrator uncalibrated() {
        System.out.println("uncalibrated: start: "+stateString());
        state = uncalibrated;
        System.out.println("uncalibrated: end: "+stateString());
        return this;
    }

    public SmartCameraExtrinsicsCalibrator calibrated() {
        System.out.println("calibrated: start: "+stateString());
        state = calibrated;
        System.out.println("calibrated: end: "+stateString());
        return this;
    }
}
