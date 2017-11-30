package william.chamberlain.androidvosopencvros;

import com.instacart.library.truetime.TrueTime;

import java.util.ArrayList;
import java.util.List;

import actionlib_msgs.GoalStatus;
import geometry_msgs.PoseWithCovarianceStamped;
import georegression.struct.se.Se3_F64;
import william.chamberlain.androidvosopencvros.ros_types.RosTypes;

import org.ros.rosjava_geometry.FrameTransform;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import static actionlib_msgs.GoalStatus.ABORTED;
import static actionlib_msgs.GoalStatus.ACTIVE;
import static actionlib_msgs.GoalStatus.LOST;
import static actionlib_msgs.GoalStatus.PENDING;
import static actionlib_msgs.GoalStatus.PREEMPTED;
import static actionlib_msgs.GoalStatus.PREEMPTING;
import static actionlib_msgs.GoalStatus.RECALLED;
import static actionlib_msgs.GoalStatus.RECALLING;
import static actionlib_msgs.GoalStatus.REJECTED;
import static actionlib_msgs.GoalStatus.SUCCEEDED;
import static william.chamberlain.androidvosopencvros.PlanningStrategy.fixedSet;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.RobotEnterFromImageSide.NaN;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.RobotEnterFromImageSide.left;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.RobotEnterFromImageSide.right;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.SelfState.calibrated;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.SelfState.robotMoving;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.SelfState.uncalibrated;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.SelfState.waitingForObs;
import static william.chamberlain.androidvosopencvros.PlanningStrategy.carryOn;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.SelfState.recordObs;

/**
 * Created by will on 24/11/17.
 */
// Ask the robot to move, so we can
// 1) measure the pixel position (2D point)
// 2) request the robot's pose (3D point)
// 3) once we have enough 2D-3D correspondences, run solvePnP to estimate the camera pose in the robot's metric map

public class SmartCameraExtrinsicsCalibrator implements RobotStatusChangeListener, RobotPoseListener, VisionTaskRunning {

    private ArrayList<Observation2> detectionInImages = new ArrayList<>(100);
    private ArrayList<PoseFromRobotData> robotPoses = new ArrayList<>(100);
    private List<AssociatedData> associatedData = new ArrayList<>();

    /*****************************************************************/
    private RobotGoalPublisher robotGoalPublisher;

    /*** RobotStatusChangeListener **************************************************************/
    @Override
    public void robotStatusChange(java.util.Date statusTime, List<GoalStatus> statuses) {
        synchronized (FSM_LOCK) {
            try {
                FSM_IS_BUSY = true;
//        System.out.println("SCEC: robotStatusChange: start: "+stateString());
        if(null != statuses && 0 < statuses.size()) {
//            System.out.println("SCEC: robotStatusChange: statuses.size()=="+statuses.size());
            GoalStatus status = statuses.get(0);
            final byte status_now = status.getStatus();
//            System.out.println("SCEC: robotStatusChange: status_now = "+status_now+", statuses.size()=="+statuses.size());
            if(last_robot_status != status_now ) {
                last_robot_status = status_now;
                switch (status_now) {
                    case PENDING:       // started waiting for planning
                        System.out.println("SCEC: robotStatusChange: PENDING");
                        robotIsMoving();
                        break;
                    case ACTIVE:        // started moving
                        System.out.println("SCEC: robotStatusChange: ACTIVE");
                        robotIsMoving();
                        break;
                    case PREEMPTED:
                        System.out.println("SCEC: robotStatusChange: PREEMPTED");
                        robotInUncertainMoveState();
                        break;
                    case SUCCEEDED:     // got there: now measure position and try estimating a ground plane
                        System.out.println("SCEC: robotStatusChange: SUCCEEDED");
                        robotFinishedMoving();
                        break;
                    case ABORTED:
                        System.out.println("SCEC: robotStatusChange: ABORTED");
                        robotInUncertainMoveState();
                        break;
                    case REJECTED:
                        System.out.println("SCEC: robotStatusChange: REJECTED");
                        robotInUncertainMoveState();
                        break;
                    case PREEMPTING:
                        System.out.println("SCEC: robotStatusChange: PREEMPTING");
                        robotInUncertainMoveState();
                        break;
                    case RECALLING:
                        System.out.println("SCEC: robotStatusChange: RECALLING");
                        robotInUncertainMoveState();
                        break;
                    case RECALLED:
                        System.out.println("SCEC: robotStatusChange: RECALLED");
                        robotInUncertainMoveState();
                        break;
                    case LOST:
                        System.out.println("SCEC: robotStatusChange: LOST");
                        robotInUncertainMoveState();
                        break;
                    default:            // failed in some fashion: go back to previous and try another
                        System.out.println("SCEC: robotStatusChange: default " + status_now);
                }
            }
        } else {

        }
//        System.out.println("SCEC: robotStatusChange: end: "+stateString());
            } finally {
                FSM_IS_BUSY = false;
            }
        }
    }

    /*** RobotPoseListener **************************************************************/
    private List<String> robotIds = null;

    /**
     * Just to keep everything wrapped up together.
     */
    private class PoseFromRobotData {
        String robotId; java.util.Date poseTime; PoseWithCovarianceStamped pose; Transform poseAsTransform; private boolean isATransform = false;
        PoseFromRobotData(String robotId_, java.util.Date poseTime_, PoseWithCovarianceStamped pose_) {
            robotId = robotId_; poseTime = poseTime_; pose = pose_;
            isATransform = false;
        }
        PoseFromRobotData(String robotId_, java.util.Date poseTime_, Transform poseAsTransform_) {
            robotId = robotId_; poseTime = poseTime_; poseAsTransform = poseAsTransform_;
            isATransform = true;
        }
        boolean isATransform() {
            return isATransform;
        }

    }

    @Override  // RobotPoseListener
    public void robotPose(String robotId, PoseWithCovarianceStamped pose) {
        robotPoses.add(new PoseFromRobotData(robotId, Date.toDate(pose.getHeader().getStamp()), pose));
    }
    private void recordRobotPose(String robotId, FrameTransform poseFrame) {
        robotPoses.add(new PoseFromRobotData(robotId, Date.toDate(poseFrame.getTime()), poseFrame.getTransform() ) );
    }

    @Override  // RobotPoseListener
    public List<String> robotIds() {
        if(null == robotIds) { robotIds = new ArrayList<>(1);  robotIds.add(""); }
        return robotIds;
    }

    /*****************************************************************/
    private final Object FSM_LOCK = new Object();
    private boolean FSM_IS_BUSY = false;
//    private void dealWithAnEvent() {
//        if(state == uncalibrated) {
//            if()
//        } else if (state == ) {
//
//        }
//    }

    /*****************************************************************/
    /**
     * Robot detected in a camera image
     * :  the 2D point is just the PixelPosition
     * :  baselink_to_tag_transform - the visual model feature pose - is needed later to transform
     *    from the robot's reported base_link pose to the 3D point of the visual feature, so that
     *    the 2D point - 3D point association is for the visual feature
     * :  the date/frameTime is the time that the visual feature/robot was detected, and is used to
     *    associate this data point with the robot's published base_link pose.
     */
    public void detectedInImage(String robotId_, java.util.Date frameTime_, PixelPosition robotPositionInImage_, Se3_F64 baselink_to_tag_transform_) {
        System.out.println("detectedInImage("+robotId_+", "+frameTime_+", "+robotPositionInImage_+", "+baselink_to_tag_transform_+");");
        if(!TrueTime.isInitialized()){
            System.out.println("SCEC: detectedInImage: drop out because cannot deal with data timestamps without NTP-corrected datetime.");
            return;
        }
        synchronized (FSM_LOCK) {
            try {
                FSM_IS_BUSY = true;
                if(uncalibrated == state) {
                    state = recordObs;
                    FrameTransform frameTransform = askRobotForItsPoseFrame();
                    Observation2 detectionInImage = new Observation2(robotId_, frameTime_, robotPositionInImage_, frameTransform.getTransform(), baselink_to_tag_transform_);
                    if(planRobotPositions(detectionInImage)) {
                        askRobotToMove();
                    }
                }
                if(waitingForObs == state) {
                    try {            System.out.println("SCEC: detectedInImage: start sleep 1.");
                        Thread.sleep(500);            System.out.println("SCEC: askRobotToMove: end sleep.");
                    } catch(InterruptedException ie) {
                        System.out.println("SCEC: detectedInImage: sleep 1 interrupted: "+ie.getMessage());
                    }
                    FrameTransform frameTransform = askRobotForItsPoseFrame();
                    Observation2 detectionInImage = new Observation2(robotId_, frameTime_, robotPositionInImage_, frameTransform.getTransform(), baselink_to_tag_transform_);
                    recordRobotDetectionInImage(detectionInImage);      System.out.println("SCEC: detectedInImage: detectionInImages.size()="+detectionInImages.size());
                    recordRobotPose(robotId_, frameTransform);          System.out.println("SCEC: detectedInImage: robotPoses.size()="+robotPoses.size());
                    PoseFromRobotData associatedPose = associateData(detectionInImage);
                    if(null!=associatedPose && estimateExtrinsics()) {
                        state = calibrated;                 System.out.println("SCEC: detectedInImage: camera calibrated");
                        return;
                    }
                    if(planRobotPositions(detectionInImage)) {
                        try {            System.out.println("SCEC: detectedInImage: start sleep 2.");
                            Thread.sleep(500);            System.out.println("SCEC: askRobotToMove: end sleep.");
                        } catch(InterruptedException ie) {
                            System.out.println("SCEC: detectedInImage: sleep 2 interrupted: "+ie.getMessage());
                        }
                        askRobotToMove();
                    }
                }
            } finally {
                FSM_IS_BUSY = false;
            }
        }
    }

    private void recordRobotDetectionInImage(Observation2 detectionInImage) {
        detectionInImages.add(detectionInImage);
    }

    /** Don't need this yet - records each frame from the camera, for timekeeping/event monitoring. */
    public void imageReceived() {
    }

    /** May return null. */
    public PoseFromRobotData associateData(Observation2 detectionInImageData_) {
        long detectionTime = detectionInImageData_.frameTime.getTime();
        long detectionTimeEarly = detectionTime - 1000;      // 0.1s
        long detectionTimeLate = detectionTime + 1000;       // 0.1s
        long bestDiffMilliseconds = 1000;                   // 1.0s
        PoseFromRobotData associatedPose = null;
        int robotPosesSize = robotPoses.size();
        for (int i_=robotPosesSize-1; i_>=0; i_--) {                // most recent into the past ...
            PoseFromRobotData poseFromRobot = robotPoses.get(i_);
            long poseFromRobotTime = poseFromRobot.poseTime.getTime();
//            if(detectionTimeLate < poseFromRobotTime) {             // ... so if our upper bound is lower than/before the pose time, ...
//                break;                                              // ... we've gone past the window.
//            }
            if(detectionTimeEarly <= poseFromRobotTime && detectionTimeLate >= poseFromRobotTime) {  //  pose time is in the bracket for the visual observation time
                long diffMilliseconds = java.lang.Math.abs( detectionTime - poseFromRobotTime );
                if (diffMilliseconds < bestDiffMilliseconds) {
                    bestDiffMilliseconds = diffMilliseconds;
                    associatedPose = poseFromRobot;
                }
            }
        }
        if(null != associatedPose) {        System.out.println("SCEC: associateData: associated="+associatedPose+" , "+detectionInImageData_);
            associatedData.add(new AssociatedData(associatedPose, detectionInImageData_));
            System.out.println("SCEC: associateData: total number associated="+associatedData.size());
        } else {System.out.println("SCEC: no association for "+detectionInImageData_);}
        return associatedPose;
    }

    private class AssociatedData {
        PoseFromRobotData    poseFromRobotData;
        Observation2         detectionInImageData;
        AssociatedData(PoseFromRobotData poseFromRobotData_, Observation2 detectionInImageData_) {
            this.poseFromRobotData = poseFromRobotData_;
            this.detectionInImageData = detectionInImageData_;
        }
    }


    /*****************************************************************/
    /* http://wiki.ros.org/amcl?distro=indigo   :   amcl_pose (geometry_msgs/PoseWithCovarianceStamped) :  Robot's estimated pose in the map, with covariance. */

    private RobotPoseMeasure robotPoseMeasure;

    void setRobotPoseMeasure(RobotPoseMeasure robotPoseMeasure_) {
        this.robotPoseMeasure = robotPoseMeasure_;
    }

    //  1) time  2) pose  3) covariance/uncertainty
    private FrameTransform askRobotForItsPoseFrame() {                          System.out.println("SCEC: askRobotForItsPose: "+stateString());
        FrameTransform transform = robotPoseMeasure.askRobotForPoseFrame();       System.out.println("SCEC: askRobotForItsPose: transform ="+transform);
        return transform;
    }
    /*****************************************************************/
    void setRobotGoalPublisher(RobotGoalPublisher robotGoalPublisher) {
        this.robotGoalPublisher = robotGoalPublisher;
    }
    /*****************************************************************/
    enum SelfState { uncalibrated, calibrated, robotMoving, waitingForObs, recordObs }

    private SelfState state = uncalibrated; // robotMoving;  // fixed poses - temp - Thu 2017_11_30  // uncalibrated;

    private String stateString() {
        return "state=" + state;
    }

    SmartCameraExtrinsicsCalibrator uncalibrated() {             System.out.println("SCEC: uncalibrated: start: "+stateString());
        //        observations = new ArrayList<>(4);  //        robotGoalPoses = new ArrayList<>(1);
        state = uncalibrated;                                           System.out.println("SCEC: uncalibrated: end: "+stateString());
        return this;
    }

    public SmartCameraExtrinsicsCalibrator calibrated() {               System.out.println("SCEC: calibrated: start: "+stateString());
        state = calibrated;                                             System.out.println("SCEC: calibrated: end: "+stateString());
        return this;
    }



    private List<Transform> robotGoalPoses = new ArrayList<>(1);

    private byte last_robot_status = PENDING;

    private final boolean askRobotToMove_worldFrame = true;

    private void askRobotToMove() {               // System.out.println("SCEC: askRobotToMove: start: "+stateString());         // too verbose
        Transform nextPose = robotGoalPoses.remove(0);
        if (askRobotToMove_worldFrame) {
            last_robot_status = -1;
            robotGoalPublisher.sendRobotGoalInWorldFrame(nextPose);       System.out.println("SCEC: askRobotToMove: sendRobotGoalInWorldFrame("+nextPose+"): "+stateString());
        } else {
            last_robot_status = -1;
            robotGoalPublisher.sendRobotGoalInRobotFrame(nextPose);
            System.out.println("SCEC: askRobotToMove: sendRobotGoalInRobotFrame("+nextPose+"): "+stateString());
        }
        try {            System.out.println("SCEC: askRobotToMove: start sleep.");
            Thread.sleep(1500);            System.out.println("SCEC: askRobotToMove: end sleep.");
        } catch(InterruptedException ie) {
            System.out.println("SCEC: askRobotToMove: sleep interrupted: "+ie.getMessage());
        }
        state = robotMoving;
        //   System.out.println("SCEC: askRobotToMove: end: "+stateString());       // too verbose
    }



    private int TEMP_NUM_OBS_EQUAL_NUM_PLANNED = -9000;
    private boolean planRobotPositions(Observation2 lastObservation) {
        if(null != robotGoalPoses && robotGoalPoses.size() > 0) {
            System.out.println("SCEC: planRobotPositions: already have goal poses: "+stateString());
            return true;
        } else {
//            if (null==observations || observations.size()<=0) {
//                System.out.println("SCEC: planRobotPositions: no observations: cannot calculate pose: "+stateString());
//                return false;
//            }
            System.out.println("SCEC: planRobotPositions: calculating poses: "+stateString());
            float scale = 0.25f;
//            Observation lastObservation = observations.get(observations.size() - 1);
            System.out.println("SCEC: planRobotPositions: lastObservation="+lastObservation);
            PixelPosition pixelPosition = lastObservation.pixelPosition;
            initialiseSideOfImageRobotEntered(pixelPosition);
            Quaternion robotGoalRotation = RosTypes.copyQuaternion(lastObservation.map_to_baselink_pose);
            Vector3 robotGoalPositionInWorldFrame = RosTypes.copyVector3(lastObservation.map_to_baselink_pose);
            robotGoalPoses = new ArrayList<Transform>(1);

            final PlanningStrategy planningStrategy = fixedSet;
            switch(planningStrategy) {
                case fixedSet: {
                    robotGoalPoses.add(new Transform(new Vector3( 2.93664726515d , 0.657309564265d , 0.0d  ),
                            new Quaternion( 0.0d , 0.0d , -0.835412750453d , 0.549623085742d )));
                    robotGoalPoses.add(new Transform(new Vector3( 2.73789962049d , 0.212102443025d , 0.0d ),
                            new Quaternion( 0.0d , 0.0d , -0.834380124658d , 0.55118944799d )));

                    robotGoalPoses.add(new Transform(new Vector3( 2.72652555835d , -0.382089352283d , 0.0d ),
                            new Quaternion( 0.0d , 0.0d , -0.835113109967d , 0.550078261306d )));
                    robotGoalPoses.add(new Transform(new Vector3( 1.86905626965d , -0.393194704328d , 0.0d ),
                            new Quaternion( 0.0d , 0.0d , -0.835113109967d , 0.550078261306d )));

//                    robotGoalPoses.add(new Transform(new Vector3( 2.13901497273d , -0.223762838519d , 0.0d ),
//                            new Quaternion( 0.0d , 0.0d , -0.835113109967d , 0.550078261306d )));
//                    robotGoalPoses.add(new Transform(new Vector3( 2.13584183641d , -0.723211302063d , 0.0d ),
//                            new Quaternion( 0.0d , 0.0d , -0.49839705985d , 0.866948885883d )));
//                    robotGoalPoses.add(new Transform(new Vector3( 2.1107298702d , -1.24241682654d , 0.0d ),
//                            new Quaternion( 0.0d , 0.0d , -0.770906821119d , 0.636947935982d )));

                    robotGoalPoses.add(new Transform(new Vector3( 1.85881299556d , -0.8d , 0.0d ),
                            new Quaternion( 0.0d , 0.0d , 0.991288357544d , 0.131709499271d )));
//                    robotGoalPoses.add(new Transform(new Vector3( 1.56498959688d , -1.13657048196d , 0.0d ),
//                            new Quaternion( 0.0d , 0.0d , 0.82470298425d , 0.565566077279d )));
                    robotGoalPoses.add(new Transform(new Vector3( 1.58425454073d , -0.685910252583d , 0.0d ),
                            new Quaternion( 0.0d , 0.0d , 0.702297407811d , 0.711883663938d )));
                    robotGoalPoses.add(new Transform(new Vector3( 1.4927118636d , -0.253677811415d , 0.0d ),
                            new Quaternion( 0.0d , 0.0d , 0.820594714969d , 0.571510554377d )));
                    robotGoalPoses.add(new Transform(new Vector3( 1.65723693733d , 0.0744799341529d , 0.0d ),
                            new Quaternion( 0.0d , 0.0d , -0.300541749254d , 0.953768660082d )));
                                    System.out.println("SCEC: planRobotPositions: fixedSet: ");
                    TEMP_NUM_OBS_EQUAL_NUM_PLANNED = robotGoalPoses.size();
                    break;
                }
                case grid: {
                    for (int x = -1; x <= 1; x++) {
                        for (int y = -1; y <= 1; y++) {
                            Vector3 changeFromFirstObservation = new Vector3((double) x * scale, (double) y * scale, 0);
                            robotGoalPositionInWorldFrame = robotGoalPositionInWorldFrame.add(changeFromFirstObservation);
                            Transform robotGoalPose = new Transform(robotGoalPositionInWorldFrame, robotGoalRotation);
                            robotGoalPoses.add(robotGoalPose);
                            System.out.println("SCEC: planRobotPositions: grid: new robotGoalPose=" + robotGoalPose);
                        }
                    }
                    TEMP_NUM_OBS_EQUAL_NUM_PLANNED = robotGoalPoses.size();
                    break;
                }
                case fixedSetOfTwo: {
                    Vector3 fixedPosition = new Vector3(1.7, -3.25, 0);
                    Transform robotGoalPose = new Transform(fixedPosition, robotGoalRotation);
                    robotGoalPoses.add(robotGoalPose);
                    System.out.println("SCEC: planRobotPositions: fixedSetOfTwo: new robotGoalPose=" + robotGoalPose);
                    fixedPosition = new Vector3(2.09, -2.25, 0);
                    robotGoalPose = new Transform(fixedPosition, robotGoalRotation);
                    robotGoalPoses.add(robotGoalPose);
                    System.out.println("SCEC: planRobotPositions: fixedSetOfTwo: new robotGoalPose=" + robotGoalPose);
                    TEMP_NUM_OBS_EQUAL_NUM_PLANNED = robotGoalPoses.size();
                    break;
                }
                case carryOn: {
                    if (askRobotToMove_worldFrame) {
                        for (int i_ = 1; i_ < 6; i_++) {
                            double y = 0.0;
                            if(i_%2 == 0) { y = (double) i_ * scale; }
                            Vector3 changeFromFirstObservationInRobotFrame = new Vector3((double) i_ * scale, y, 0.0);
                            Vector3 changeFromFirstObservationInWorldFrame = robotGoalRotation.rotateAndScaleVector(changeFromFirstObservationInRobotFrame);
                            robotGoalPositionInWorldFrame = robotGoalPositionInWorldFrame.add(changeFromFirstObservationInWorldFrame);
                            Transform robotGoalPose = new Transform(robotGoalPositionInWorldFrame, robotGoalRotation);
                            robotGoalPoses.add(robotGoalPose);
                            System.out.println("SCEC: planRobotPositions: carryOnInWorldFrame: new robotGoalPose=" + robotGoalPose);
                        }
                        TEMP_NUM_OBS_EQUAL_NUM_PLANNED = robotGoalPoses.size();
                    } else {
                        for (int i_ = 1; i_ < 6; i_++) {
                            double y = 0.0;
                            if(i_%2 == 0) { y = (double) i_ * scale; }
                            Vector3 changeFromFirstObservationInRobotFrame = new Vector3((double) i_ * scale, y, 0.0);
                            Transform robotGoalPose = new Transform(changeFromFirstObservationInRobotFrame, Quaternion.identity());
                            robotGoalPoses.add(robotGoalPose);
                            System.out.println("SCEC: planRobotPositions: carryOnInRobotFrame: new robotGoalPose=" + robotGoalPose);
                        }
                        TEMP_NUM_OBS_EQUAL_NUM_PLANNED = robotGoalPoses.size();
                    }
                    break;
                }
            }

            return true;
        }
    }

    enum RobotEnterFromImageSide { NaN, left, right }
    private RobotEnterFromImageSide robotEnteredFromSide = NaN;
    private void initialiseSideOfImageRobotEntered(PixelPosition pixelPosition) {
        if(robotEnteredFromSide == NaN) {            System.out.println("SCEC: initialiseSideOfImageRobotEntered: robotEnteredFromSide = "+robotEnteredFromSide);
            if (pixelPosition.getU() > pixelPosition.getWidth() / 2.0) {  // right of image
                robotEnteredFromSide = right;
            } else {    // left of image
                robotEnteredFromSide = left;
            }
            System.out.println("SCEC: initialiseSideOfImageRobotEntered: robotEnteredFromSide = "+robotEnteredFromSide);
        }
    }


    private int robotFinishedMoving_waitCount = 0;
    private final static int FPS_EST         = 5;
    private final static int SECONDS_TO_WAIT = 5;


    private void robotIsMoving() {
        System.out.println("SCEC: robotIsMoving: "+stateString());
    }

    private void robotInUncertainMoveState(){
        System.out.println("SCEC: !!: robotInUncertainMoveState: "+stateString());
    }

    protected void robotFinishedMoving() {
        System.out.println("SCEC: robotFinishedMoving: start: "+stateString());
        synchronized (FSM_LOCK) {
            try {
                FSM_IS_BUSY = true;
                if (state == robotMoving) {
                    try {            System.out.println("SCEC: robotFinishedMoving: start sleep.");
                        Thread.sleep(1500);            System.out.println("SCEC: askRobotToMove: end sleep.");
                    } catch(InterruptedException ie) {
                        System.out.println("SCEC: robotFinishedMoving: sleep interrupted: "+ie.getMessage());
                    }
                    System.out.println("SCEC: robotFinishedMoving: end sleep.");
                    state = waitingForObs;
                    System.out.println("SCEC: robotFinishedMoving: set to waitingForObs: " + stateString());
                    robotFinishedMoving_waitCount = 0;
                }
            } finally {
                FSM_IS_BUSY = false;
            }
        }
        System.out.println("SCEC: robotFinishedMoving: end: "+stateString());
    }

    /** @return true if the extrinsics have been calculated and can finish the process.*/
    private boolean estimateExtrinsics() {
        int i_ = 0;
        for(AssociatedData associatedDatapoint : associatedData) {
            System.out.println("SCEC: estimateExtrinsics: observation "+i_+" = "+associatedDatapoint);
            i_++;
        }
        if(null != associatedData  &&  associatedData.size() >= TEMP_NUM_OBS_EQUAL_NUM_PLANNED) {
            System.out.println("SCEC: estimateExtrinsics: TRUE");
            return true;
        }
        System.out.println("SCEC: estimateExtrinsics: false");
        return false;
    }

}

enum PlanningStrategy { fixedSetOfTwo, fixedSet, carryOn, grid }
