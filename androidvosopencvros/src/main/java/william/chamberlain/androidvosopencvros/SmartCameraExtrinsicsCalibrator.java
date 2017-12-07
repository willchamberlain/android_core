package william.chamberlain.androidvosopencvros;

import java.util.ArrayList;
import java.util.List;

import actionlib_msgs.GoalStatus;
import geometry_msgs.PoseWithCovarianceStamped;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Quaternion_F64;
import william.chamberlain.androidvosopencvros.ros_types.RosTypes;

import org.bytedeco.javacpp.opencv_calib3d;
import org.bytedeco.javacpp.opencv_core;
import org.ejml.data.DenseMatrix64F;
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
import static org.bytedeco.javacpp.opencv_core.CV_64FC1;
import static org.bytedeco.javacpp.opencv_core.setIdentity;
import static william.chamberlain.androidvosopencvros.Geometry_OpenCV.matToString;
import static william.chamberlain.androidvosopencvros.PlanningStrategy.fixedSet;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.RobotEnterFromImageSide.NaN;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.RobotEnterFromImageSide.left;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.RobotEnterFromImageSide.right;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.SelfState.calibrated;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.SelfState.robotMoving;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.SelfState.uncalibrated;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.SelfState.waitingForObs;
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

        @Override
        public String toString() {
            return "PoseFromRobotData{" +
                    "robotId='" + robotId + '\'' +
                    ", poseTime=" + poseTime +
                    ", poseTime=" + poseTime.getTime() +"ms" +
                    ", pose=" + (isATransform ? "none": pose.toString())+
                    ", poseAsTransform=" + (isATransform ? poseAsTransform.toString() : "none") +
                    ", isATransform=" + isATransform +
                    '}';
        }
    }

    @Override  // RobotPoseListener
    public void robotPose(String robotId, PoseWithCovarianceStamped pose) {
        robotPoses.add(new PoseFromRobotData(robotId, DateAndTime.toJavaDate(pose.getHeader().getStamp()), pose));
    }
    public void recordRobotPose(String robotId, FrameTransform poseFrame) {
        robotPoses.add(new PoseFromRobotData(robotId, DateAndTime.toJavaDate(poseFrame.getTime()), poseFrame.getTransform() ) );
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
     * :  the date/imageCaptureTime is the time that the visual feature/robot was detected, and is used to
     *    associate this data point with the robot's published base_link pose.
     */
    public Se3_F64 detectedInImage(String robotId_, java.util.Date imageCaptureTime_, PixelPosition robotPositionInImage_, Se3_F64 baselink_to_tag_transform_, FrameTransform frameTransform) {
        return detectedInImage_2(robotId_, imageCaptureTime_, robotPositionInImage_, baselink_to_tag_transform_, frameTransform);
    }

    public Se3_F64 detectedInImage_2(String robotId_, java.util.Date imageCaptureTime_, PixelPosition robotPositionInImage_, Se3_F64 baselink_to_tag_transform_, FrameTransform frameTransform) {
        System.out.println("detectedInImage_2("+robotId_+", "+imageCaptureTime_+", "+robotPositionInImage_+", "+baselink_to_tag_transform_+");");
        if(calibrated == state) { System.out.println("detectedInImage_2: ALREADY CALIBRATED"); }
        TEMP_NUM_OBS_EQUAL_NUM_PLANNED = 20;
//        FrameTransform frameTransform = askRobotForItsPoseFrame();
        recordRobotPose(robotId_, frameTransform);
        Observation2 detectionInImage;

        detectionInImage = new Observation2(robotId_, DateAndTime.toJavaDate(frameTransform.getTime()), robotPositionInImage_, frameTransform.getTransform(), baselink_to_tag_transform_);
        recordRobotDetectionInImage(detectionInImage);      System.out.println("SCEC: detectedInImage_2: detectionInImages.size()="+detectionInImages.size());

        detectionInImage = new Observation2(robotId_, imageCaptureTime_, robotPositionInImage_, frameTransform.getTransform(), baselink_to_tag_transform_);
        recordRobotDetectionInImage(detectionInImage);      System.out.println("SCEC: detectedInImage_2: detectionInImages.size()="+detectionInImages.size());
        System.out.println("SCEC: detectedInImage_2: Observation2 imageCaptureTime_ = "+imageCaptureTime_.getTime()+"ms");
        System.out.println("SCEC: detectedInImage_2: detectionInImage imageCaptureTime = "+detectionInImage.imageCaptureTime.getTime()+"ms");

        associateData(detectionInImage);
        Se3_F64 estimatedCameraPose = estimateExtrinsics();
        if(null != estimatedCameraPose){
            state = calibrated;
            System.out.println("SCEC: detectedInImage_2: estimateExtrinsics()!!");
            return estimatedCameraPose;
        } else {
            System.out.println("SCEC: detectedInImage_2: NOT estimateExtrinsics() yet.");
            return null;
        }
    }

    private void recordRobotDetectionInImage(Observation2 detectionInImage) {
        detectionInImages.add(detectionInImage);
    }

    /** Don't need this yet - records each frame from the camera, for timekeeping/event monitoring. */
    public void imageReceived() {
    }

    static final int TIME_TOLERANCE_MS = 200;

    /** May return null. */
    public PoseFromRobotData associateData(Observation2 detectionInImageData_) {
        long detectionTime = detectionInImageData_.imageCaptureTime.getTime();
        long detectionTimeEarly = detectionTime - TIME_TOLERANCE_MS;      // 0.1s
        long detectionTimeLate  = detectionTime + TIME_TOLERANCE_MS;       // 0.1s
        long bestDiffMilliseconds = 1000000;                   // 1.0s
        PoseFromRobotData associatedPose = null;
        int robotPosesSize = robotPoses.size();
        for (int i_=robotPosesSize-1; i_>=0; i_--) {                // most recent into the past ...
            PoseFromRobotData poseFromRobot = robotPoses.get(i_);
            long poseFromRobotTime = poseFromRobot.poseTime.getTime();
//            if(detectionTimeLate < poseFromRobotTime) {             // ... so if our upper bound is lower than/before the pose time, ...
//                break;                                              // ... we've gone past the window.
//            }
            System.out.println("associateData: diff="+(detectionTime-poseFromRobotTime)+", detectionTimeEarly="+detectionTimeEarly+", poseFromRobotTime="+poseFromRobotTime+", detectionTime="+detectionTime+", detectionTimeLate="+detectionTimeLate);
            if(detectionTimeEarly <= poseFromRobotTime && detectionTimeLate >= poseFromRobotTime) {  //  pose time is in the bracket for the visual observation time
                System.out.println("associateData: IS in range: diff="+(detectionTime-poseFromRobotTime)+", detectionTimeEarly="+detectionTimeEarly+", poseFromRobotTime="+poseFromRobotTime+", detectionTime="+detectionTime+", detectionTimeLate="+detectionTimeLate);
                long diffMilliseconds = java.lang.Math.abs( detectionTime - poseFromRobotTime );
                if (diffMilliseconds < bestDiffMilliseconds) {
                    bestDiffMilliseconds = diffMilliseconds;
                    associatedPose = poseFromRobot;
                    System.out.println("associateData: current best: diff="+(detectionTime-poseFromRobotTime)+", detectionTimeEarly="+detectionTimeEarly+", poseFromRobotTime="+poseFromRobotTime+", detectionTime="+detectionTime+", detectionTimeLate="+detectionTimeLate);
                }
            } else {
                System.out.println("associateData: NOT in range: diff="+(detectionTime-poseFromRobotTime)+", detectionTimeEarly="+detectionTimeEarly+", poseFromRobotTime="+poseFromRobotTime+", detectionTime="+detectionTime+", detectionTimeLate="+detectionTimeLate);
            }
        }
        if(null != associatedPose) {        System.out.println("SCEC: associateData: associated="+associatedPose+" , "+detectionInImageData_);
            synchronized (associatedData_lock) {
                associatedData.add(new AssociatedData(associatedPose, detectionInImageData_));
                System.out.println("SCEC: associateData: total number associated=" + associatedData.size());
                System.out.println("associateData: best: diff=" + (detectionTime - associatedPose.poseTime.getTime()) + ", detectionTimeEarly=" + detectionTimeEarly + ", poseFromRobotTime=" + associatedPose.poseTime.getTime() + ", detectionTime=" + detectionTime + ", detectionTimeLate=" + detectionTimeLate);
                System.out.println("associateData: matched       Observation2: " + detectionInImageData_.toString());
                System.out.println("associateData: matched  PoseFromRobotData: " + associatedPose.toString());

                // now, do the transformation from base_link to the visual feature
            }


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
    public FrameTransform askRobotForItsPoseFrame() {                          System.out.println("SCEC: askRobotForItsPose: "+stateString());
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

    private final Object associatedData_lock = new Object();

    /** @return true if the extrinsics have been calculated and can finish the process.*/
    private Se3_F64 estimateExtrinsics() {
        int i_ = 0;
        /*
            if(null != associatedData  &&  associatedData.size() >= TEMP_NUM_OBS_EQUAL_NUM_PLANNED) {
                System.out.println("SCEC: estimateExtrinsics: TRUE");
                return true; }
         */
        synchronized (associatedData_lock) {
            if(null != associatedData  &&  associatedData.size() >= TEMP_NUM_OBS_EQUAL_NUM_PLANNED) {
                IMAGE_PIXEL_2D_DATA_POINTS = new double[associatedData.size() * 2];
                WORLD_3D_DATA_POINTS = new double[associatedData.size() * 3];
                for (AssociatedData associatedDatapoint : associatedData) {
                    System.out.println("SCEC: estimateExtrinsics: AssociatedData " + i_ + " = " + associatedDatapoint);
                    IMAGE_PIXEL_2D_DATA_POINTS[i_ + 0] = associatedDatapoint.detectionInImageData.pixelPosition.getU();
                    IMAGE_PIXEL_2D_DATA_POINTS[i_ + 1] = associatedDatapoint.detectionInImageData.pixelPosition.getV();

                    WORLD_3D_DATA_POINTS[i_ + 0] = associatedDatapoint.detectionInImageData.map_to_tag_transform_boofcv.getT().getX();
                    WORLD_3D_DATA_POINTS[i_ + 1] = associatedDatapoint.detectionInImageData.map_to_tag_transform_boofcv.getT().getY();
                    WORLD_3D_DATA_POINTS[i_ + 2] = associatedDatapoint.detectionInImageData.map_to_tag_transform_boofcv.getT().getZ();

                    i_++;
                }
                opencv_core.Mat transformOpenCVMat = calculateExtrinsics();

                Se3_F64 transformBoofCVMat      = new Se3_F64();

                transformBoofCVMat.setTranslation(
                    transformOpenCVMat.getDoubleBuffer().get(3),
                    transformOpenCVMat.getDoubleBuffer().get(7),
                    transformOpenCVMat.getDoubleBuffer().get(11));

                DenseMatrix64F rotDense = new DenseMatrix64F(3,3);
                    rotDense.set(0,0, transformOpenCVMat.getDoubleBuffer().get(0));
                    rotDense.set(0,1, transformOpenCVMat.getDoubleBuffer().get(1));
                    rotDense.set(0,2, transformOpenCVMat.getDoubleBuffer().get(2));
                    rotDense.set(1,0, transformOpenCVMat.getDoubleBuffer().get(4));
                    rotDense.set(1,1, transformOpenCVMat.getDoubleBuffer().get(5));
                    rotDense.set(1,2, transformOpenCVMat.getDoubleBuffer().get(6));
                    rotDense.set(2,0, transformOpenCVMat.getDoubleBuffer().get(8));
                    rotDense.set(2,1, transformOpenCVMat.getDoubleBuffer().get(9));
                    rotDense.set(2,2, transformOpenCVMat.getDoubleBuffer().get(10));
                transformBoofCVMat.setRotation(rotDense);

                Quaternion_F64 quaternionBoofCV = new Quaternion_F64();
                ConvertRotation3D_F64.matrixToQuaternion(transformBoofCVMat.getR(), quaternionBoofCV);

                System.out.println("SCEC: estimateExtrinsics: TRUE");
                return transformBoofCVMat;
            }
        }
        System.out.println("SCEC: estimateExtrinsics: false");
        return null;
    }

    /**
     * Robot detected in a camera image
     * :  the 2D point is just the PixelPosition
     * :  baselink_to_tag_transform - the visual model feature pose - is needed later to transform
     *    from the robot's reported base_link pose to the 3D point of the visual feature, so that
     *    the 2D point - 3D point association is for the visual feature
     * :  the date/imageCaptureTime is the time that the visual feature/robot was detected, and is used to
     *    associate this data point with the robot's published base_link pose.
     */
    public void detectedInImage_1(String robotId_, java.util.Date frameTime_, PixelPosition robotPositionInImage_, Se3_F64 baselink_to_tag_transform_) {
        System.out.println("detectedInImage_1("+robotId_+", "+frameTime_+", "+robotPositionInImage_+", "+baselink_to_tag_transform_+");");
        //        if(!TrueTime.isInitialized()){
        //            System.out.println("SCEC: detectedInImage_1: drop out because cannot deal with data timestamps without NTP-corrected datetime.");
        //            return;
        //        }
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
                    try {            System.out.println("SCEC: detectedInImage_1: start sleep 1.");
                        Thread.sleep(500);            System.out.println("SCEC: askRobotToMove: end sleep.");
                    } catch(InterruptedException ie) {
                        System.out.println("SCEC: detectedInImage_1: sleep 1 interrupted: "+ie.getMessage());
                    }
                    FrameTransform frameTransform = askRobotForItsPoseFrame();
                    Observation2 detectionInImage = new Observation2(robotId_, frameTime_, robotPositionInImage_, frameTransform.getTransform(), baselink_to_tag_transform_);
                    recordRobotDetectionInImage(detectionInImage);      System.out.println("SCEC: detectedInImage_1: detectionInImages.size()="+detectionInImages.size());
                    recordRobotPose(robotId_, frameTransform);          System.out.println("SCEC: detectedInImage_1: robotPoses.size()="+robotPoses.size());
                    PoseFromRobotData associatedPose = associateData(detectionInImage);
                    if(null!=associatedPose) {
                        Se3_F64 estimatedCameraPose = estimateExtrinsics();
                        if(null != estimatedCameraPose) {
                            state = calibrated;
                            System.out.println("SCEC: detectedInImage_1: camera calibrated");
                            return;
                        }
                    }
                    if(planRobotPositions(detectionInImage)) {
                        try {            System.out.println("SCEC: detectedInImage_1: start sleep 2.");
                            Thread.sleep(500);            System.out.println("SCEC: askRobotToMove: end sleep.");
                        } catch(InterruptedException ie) {
                            System.out.println("SCEC: detectedInImage_1: sleep 2 interrupted: "+ie.getMessage());
                        }
                        askRobotToMove();
                    }
                }
            } finally {
                FSM_IS_BUSY = false;
            }
        }
    }



    private static final double[] CAMERA_INTRINSICS_MATRIX = {
            2873.9d, 0.0d, 1624.4d,
            0.0d, 2867.8d, 921.6d,
            0.0d, 0.0d, 1.0d};

    private static final double[] CAMERA_DISTORTION_COEFFICIENTS_5 = {0.0748d, -0.1524d, 0.0887d, 0.0d, 0.0d};

    private double[] IMAGE_PIXEL_2D_DATA_POINTS = {
            1538d, 589d,
            1723d, 1013d };

    private double[] WORLD_3D_DATA_POINTS = new double[]{
            622.0000d, 643.5000d, 90.0000d,
            655.0000d, 643.5000d, 0.000d };

    private opencv_core.Mat calculateExtrinsics() {
        System.out.println("onCameraFrame: before opencv_calib3d.solvePnPRansac");
        opencv_core.Mat objectPoints   = new opencv_core.Mat(WORLD_3D_DATA_POINTS.length/3, 3, CV_64FC1); // 10 measurements, 3 dimensions, 1 channel: could also have Mat(10,1,3)
        objectPoints.getDoubleBuffer().put(WORLD_3D_DATA_POINTS);
        System.out.println("onCameraFrame: objectPoints = "+matToString(objectPoints));
        System.out.println("onCameraFrame: objectPoints.toString()="+objectPoints.toString());
        System.out.println("onCameraFrame: objectPoints.size().toString()="+objectPoints.size().toString());
        // i.e. a 10-element vector with each element a 3d measurement/point :
        // Nx3 1-channel or 1xN/Nx1 3-channel
        // HeadPose example uses coordinates based on the head model centre - relative to camera focal axis - [ x=right , y=up , z=depth/distance=toward-camera]
        List<opencv_core.Point2f> imagePointVector = new ArrayList<>();
        double[] pixeldata = IMAGE_PIXEL_2D_DATA_POINTS;
        opencv_core.Mat imagePoints    = new opencv_core.Mat(WORLD_3D_DATA_POINTS.length/3, 2, CV_64FC1);
        imagePoints.getDoubleBuffer().put(pixeldata);
        System.out.println("onCameraFrame: imagePoints = "+matToString(imagePoints));
        imagePoints.getDoubleBuffer().toString();
        System.out.println("onCameraFrame: imagePoints.toString()="+imagePoints.toString());
        System.out.println("onCameraFrame: imagePoints.size().toString()="+imagePoints.size().toString());
        imagePoints.size().toString();
//            org.bytedeco.javacpp.Indexer indexer = imagePoints.createIndexer();
//            indexer.put(row,col,val)
//            Mat imagePoints    = new Mat(10, 2, CV_64FC1); // pixel coordinates, as [ u=x=right, v=y=down ]
//            imagePoints.push_back(new Mat(opencv_core.Point2f(1538.0f,589.0f)));
        opencv_core.Mat cameraMatrix   = new opencv_core.Mat(3, 3, CV_64FC1);
        cameraMatrix.getDoubleBuffer().put(CAMERA_INTRINSICS_MATRIX);
        System.out.println("onCameraFrame: cameraMatrix = "+matToString(cameraMatrix));
        double[] distCoeffsArray = CAMERA_DISTORTION_COEFFICIENTS_5;
        opencv_core.Mat distCoeffs     = new opencv_core.Mat(distCoeffsArray.length, 1, CV_64FC1 );
        System.out.println("onCameraFrame: distCoeffs = "+matToString(distCoeffs));
        distCoeffs.getDoubleBuffer().put(distCoeffsArray);
        opencv_core.Mat rVec_Estimated = new opencv_core.Mat(3, 3, CV_64FC1); // Output rotation   _vector_ : world-to-camera
        opencv_core.Mat tVec_Estimated = new opencv_core.Mat(3, 1, CV_64FC1); // Output translation vector  : world-to-camera

        //        @param cameraMatrix Input camera matrix \f$A = \vecthreethree
        //              {fx}{0}{cx}
        //              {0}{fy}{cy}
        //              {0}{0}{1}\f$ .
        //        @param distCoeffs Input vector of distortion coefficients
        //             \f$( k_1, k_2, p_1, p_2  [, k_3  [, k_4, k_5, k_6  [, s_1, s_2, s_3, s_4  [, \tau_x, \tau_y] ] ] ] )\f$ of
        //        4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are
        //        assumed.
        //        @param rvec Output rotation vector (see Rodrigues ) that, together with tvec , brings points from
        //        the model coordinate system to the camera coordinate system.
        //        @param tvec Output translation vector.
        //        @Namespace("cv") public static native @Cast("bool") boolean solvePnPRansac( @ByVal Mat objectPoints, @ByVal Mat imagePoints,
        //                                  @ByVal Mat cameraMatrix, @ByVal Mat distCoeffs,
        //                                  @ByVal Mat rvec, @ByVal Mat tvec,
        //                                  @Cast("bool") boolean useExtrinsicGuess/*=false*/, int iterationsCount/*=100*/,
        //                                  float reprojectionError/*=8.0*/, double confidence/*=0.99*/,
        //                                  @ByVal(nullValue = "cv::OutputArray(cv::noArray())") Mat inliers, int flags/*=cv::SOLVEPNP_ITERATIVE*/ );
//        opencv_calib3d.solvePnPRansac(objectPoints, imagePoints,
//                cameraMatrix,   distCoeffs,
//                rVec_Estimated, tVec_Estimated,
//                false,          100,
//                0.8f,           0.99d);
        opencv_calib3d.solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rVec_Estimated, tVec_Estimated);

        System.out.println("onCameraFrame: rVec_Estimated = "+rVec_Estimated);
        System.out.println("onCameraFrame: rVec_Estimated.size().width()= "+rVec_Estimated.size().width()+", .height()="+rVec_Estimated.size().height());
        System.out.println("onCameraFrame: rVec_Estimated = "+matToString(rVec_Estimated));

        opencv_core.Mat rotationMatrix = new opencv_core.Mat(3, 3, CV_64FC1);
        opencv_calib3d.Rodrigues(rVec_Estimated,rotationMatrix);
        System.out.println("onCameraFrame: rotationMatrix = "+matToString(rotationMatrix));

        opencv_core.MatExpr rotationMatrixTransposeTemp = rotationMatrix.t();
        opencv_core.Mat rotationMatrixTranspose = new opencv_core.Mat(1,1,CV_64FC1);
        rotationMatrixTranspose.put(rotationMatrixTransposeTemp);
        System.out.println("onCameraFrame: rotationMatrixTranspose = "+matToString(rotationMatrixTranspose));

//            Mat rotationMatrixInverse      = rotationMatrixTranspose;
        opencv_core.Mat negOne = new opencv_core.Mat(3,3,CV_64FC1); setIdentity(negOne, new opencv_core.Scalar(-1.0d));
        System.out.println("onCameraFrame: negOne = "+matToString(negOne));
        negOne.getDoubleBuffer().put(new double[]{
                -1.0d,  0.0d,  0.0d,
                0.0d, -1.0d,  0.0d,
                0.0d,  0.0d, -1.0d });
        System.out.println("onCameraFrame: negOne = "+matToString(negOne));

        System.out.println("onCameraFrame: tVec_Estimated = "+tVec_Estimated);
        System.out.println("onCameraFrame: tVec_Estimated = "+matToString(tVec_Estimated));

        opencv_core.Mat one = new opencv_core.Mat(3,3,CV_64FC1); setIdentity(one, new opencv_core.Scalar(1.0d));
        System.out.println("onCameraFrame: one = "+matToString(one));
//            negOne.getDoubleBuffer().put(new double[]{
//                    1.0d,  0.0d,  0.0d,
//                    0.0d, 1.0d,  0.0d,
//                    0.0d,  0.0d, 1.0d });
        System.out.println("onCameraFrame: one = "+matToString(one));
        opencv_core.MatExpr translationInverseTemp_One = one.mul(rotationMatrixTranspose);
        System.out.println("onCameraFrame: translationInverseTemp_One.asMat() = "+matToString(translationInverseTemp_One.asMat()));
        translationInverseTemp_One = opencv_core.multiply(rotationMatrixTranspose,one);
        System.out.println("onCameraFrame: translationInverseTemp_One.asMat() = "+matToString(translationInverseTemp_One.asMat()));

        opencv_core.MatExpr translationInverseTemp = opencv_core.multiply(rotationMatrixTranspose,negOne); // element-wise
        opencv_core.Mat dummy_1 = translationInverseTemp.asMat();
        System.out.println("onCameraFrame: translationInverseTemp.asMat() = "+matToString(translationInverseTemp.asMat()));
        System.out.println("onCameraFrame: translationInverseTemp.size() width="+translationInverseTemp.size().width()+", height= "+translationInverseTemp.size().height());
        System.out.println("onCameraFrame: translationInverseTemp = "+translationInverseTemp);
        opencv_core.MatExpr translationInverseTempTemp = opencv_core.multiply(translationInverseTemp,tVec_Estimated);
        opencv_core.Mat dummy_2 = translationInverseTempTemp.asMat();
        System.out.println("onCameraFrame: translationInverseTempTemp.asMat() = "+matToString(translationInverseTempTemp.asMat()));
        System.out.println("onCameraFrame: translationInverseTempTemp.size() width="+translationInverseTempTemp.size().width()+", height= "+translationInverseTempTemp.size().height());
        System.out.println("onCameraFrame: translationInverseTempTemp = "+translationInverseTempTemp);
        opencv_core.Mat translationInverse = new opencv_core.Mat(1,1,CV_64FC1);
        System.out.println("onCameraFrame: translationInverse = "+translationInverse);
        translationInverse.put(translationInverseTempTemp);
        System.out.println("onCameraFrame: translationInverse = "+matToString(translationInverse));

        opencv_core.Mat cameraPoseInWorldCoords = new opencv_core.Mat(4, 4, CV_64FC1);
        // As in Python, start is an inclusive left boundary of the range and end is an exclusive right boundary of the range. Such a half-opened interval ...
        // 0..2, 0..2 inclusive
//            Mat camPoseTopLeft = cameraPoseInWorldCoords.apply(new opencv_core.Range(0,3), new opencv_core.Range(0,3));
//            System.out.println("onCameraFrame: camPoseTopLeft before = "+matToString(camPoseTopLeft));
//            camPoseTopLeft.getDoubleBuffer().put(rotationMatrixTranspose.getDoubleBuffer());
//            System.out.println("onCameraFrame: camPoseTopLeft = "+matToString(camPoseTopLeft));
//            System.out.println("onCameraFrame: cameraPoseInWorldCoords = "+matToString(cameraPoseInWorldCoords));  // goes sequentially, not in the shape of top left - how is this useful ??
        cameraPoseInWorldCoords.getDoubleBuffer().put(0, rotationMatrixTranspose.getDoubleBuffer().get(0)); // row one, all cols
        cameraPoseInWorldCoords.getDoubleBuffer().put(1, rotationMatrixTranspose.getDoubleBuffer().get(1)); // row one, all cols
        cameraPoseInWorldCoords.getDoubleBuffer().put(2, rotationMatrixTranspose.getDoubleBuffer().get(2)); // row one, all cols
        cameraPoseInWorldCoords.getDoubleBuffer().put(4, rotationMatrixTranspose.getDoubleBuffer().get(3));
        cameraPoseInWorldCoords.getDoubleBuffer().put(5, rotationMatrixTranspose.getDoubleBuffer().get(4));
        cameraPoseInWorldCoords.getDoubleBuffer().put(6, rotationMatrixTranspose.getDoubleBuffer().get(5));
        cameraPoseInWorldCoords.getDoubleBuffer().put(8, rotationMatrixTranspose.getDoubleBuffer().get(6));
        cameraPoseInWorldCoords.getDoubleBuffer().put(9, rotationMatrixTranspose.getDoubleBuffer().get(7));
        cameraPoseInWorldCoords.getDoubleBuffer().put(10, rotationMatrixTranspose.getDoubleBuffer().get(8));
        System.out.println("onCameraFrame: cameraPoseInWorldCoords after inserting rotation = "+matToString(cameraPoseInWorldCoords));

        // 0..2,3..3 inclusive
//            Mat camPoseRightCol = cameraPoseInWorldCoords.apply(new opencv_core.Range(0,3), new opencv_core.Range(3,4));
//            System.out.println("onCameraFrame: camPoseRightCol before = "+matToString(camPoseRightCol));
//            camPoseRightCol.getDoubleBuffer().put(translationInverse.getDoubleBuffer());
//            System.out.println("onCameraFrame: camPoseRightCol = "+matToString(camPoseRightCol));
//            System.out.println("onCameraFrame: cameraPoseInWorldCoords = "+matToString(cameraPoseInWorldCoords));  // goes sequentially, not in the shape of top left - how is this useful ??
        cameraPoseInWorldCoords.getDoubleBuffer().put(3, translationInverse.getDoubleBuffer().get(0));
        cameraPoseInWorldCoords.getDoubleBuffer().put(7, translationInverse.getDoubleBuffer().get(1));
        cameraPoseInWorldCoords.getDoubleBuffer().put(11, translationInverse.getDoubleBuffer().get(2));
        System.out.println("onCameraFrame: cameraPoseInWorldCoords after inserting translation = "+matToString(cameraPoseInWorldCoords));

// from here
        double[] homogeneousPadding = new double[]{0.0d, 0.0d, 0.0d, 1.0d};
//                // 3..3,0..3 inclusive
//            cameraPoseInWorldCoords.apply(new opencv_core.Range(3,4), new opencv_core.Range(0,4)).put(new Mat(homogeneousPadding));
//            System.out.println("onCameraFrame: cameraPoseInWorldCoords = "+matToString(cameraPoseInWorldCoords));
        cameraPoseInWorldCoords.getDoubleBuffer().put(12, homogeneousPadding[0]);
        cameraPoseInWorldCoords.getDoubleBuffer().put(13, homogeneousPadding[1]);
        cameraPoseInWorldCoords.getDoubleBuffer().put(14, homogeneousPadding[2]);
        cameraPoseInWorldCoords.getDoubleBuffer().put(15, homogeneousPadding[3]);
        System.out.println("onCameraFrame: cameraPoseInWorldCoords after padding = "+matToString(cameraPoseInWorldCoords));

        System.out.println("onCameraFrame: after opencv_calib3d.solvePnPRansac");
        return cameraPoseInWorldCoords;
    }



}


enum PlanningStrategy { fixedSetOfTwo, fixedSet, carryOn, grid }
