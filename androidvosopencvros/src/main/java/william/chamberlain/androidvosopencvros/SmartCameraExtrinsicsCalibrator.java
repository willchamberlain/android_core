package william.chamberlain.androidvosopencvros;

import android.support.annotation.NonNull;
import android.util.Log;

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
import org.opencv.core.Mat;
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
import static william.chamberlain.androidvosopencvros.Geometry_OpenCV.matToString;
import static william.chamberlain.androidvosopencvros.PlanningStrategy.fixedSet;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.RobotEnterFromImageSide.NaN;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.RobotEnterFromImageSide.left;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.RobotEnterFromImageSide.right;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.SelfState.calibrated;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.SelfState.robotMoving;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.SelfState.uncalibrated;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.SelfState.waitingForObs;

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
    private PoseEstimator2D3D poseEstimator2D3D;

    private PosedEntity posedEntity;

    /*****************************************************************/
    private FileLogger fileLogger;


    /*****************************************************************/
    private RobotGoalPublisher robotGoalPublisher;


    /*****************************************************************/
    public void recordRobotObservation(String robotId_, java.util.Date imageFrameTime_, PixelPosition pixelPosition_, Se3_F64 transformOfFeatureInVisualModel_){
        robotPoseMeasure.askRobotForPoseFrameAsync(robotId_, imageFrameTime_, pixelPosition_, transformOfFeatureInVisualModel_, this);
    }


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

//    @Override  // RobotPoseListener
//    public void robotPose(String robotId, PoseWithCovarianceStamped pose) {
//        robotPoses.add(new PoseFromRobotData(robotId, DateAndTime.toJavaDate(pose.getHeader().getStamp()), pose));
//    }
    public void recordRobotPose(String robotId, FrameTransform poseFrame) {
        robotPoses.add(new PoseFromRobotData(robotId, DateAndTime.toJavaDate(poseFrame.getTime()), poseFrame.getTransform() ) );
    }


    /**
     * time_ is the image capture time from the camera API,
     * pixelPosition_ is the feature pixel position in the image from image processing = 2D point,
     * transform_ is the robot pose from service call to C++ TF listener = 3D point.
     */
    public void robotPoseObservation(java.util.Date time_, PixelPosition pixelPosition_, Se3_F64 transformOfFeatureInVisualModel_, FrameTransform transform_) {

    //public Observation2(String robotId_, java.util.Date imageCaptureTime_, PixelPosition pixelPosition_, Transform map_to_baselink_pose_, Se3_F64 baselink_to_tag_transform_) {
        synchronized (observations_LOCK) {
            // TODO - will cause clashes between different tags - need to add tag id to the checks
            for(Observation2 obs2 : observations) {
                if ( Math.abs(obs2.pixelPosition.getU() - pixelPosition_.getU()) < 10 && Math.abs(obs2.pixelPosition.getV() - pixelPosition_.getV()) < 5 ) {
                    Observation2 observation2 = new Observation2("", time_, pixelPosition_, transform_.getTransform(), transformOfFeatureInVisualModel_);
                    System.out.println("robotPoseObservation: observation "+observation2+" not added: it is too similar to "+obs2);
                    return;
                }
            }
            Observation2 observation2 = new Observation2("", time_, pixelPosition_, transform_.getTransform(), transformOfFeatureInVisualModel_);
            observations.add(observation2);
            try {
                logObservations2ToFile();
            } catch (Throwable e) {
                System.out.println("ERROR:  robotPoseObservation: exception logging Observation2 list to file; exception in in logObservations2ToFile()"+e.getMessage());
                e.printStackTrace();
            }
        }
    }

    List<Observation2> observations = new ArrayList<>();

    static Object observations_LOCK = new Object();





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
    public Se3_F64 detectedInImage(
            String robotId_, java.util.Date imageCaptureTime_, PixelPosition robotPositionInImage_
            , Se3_F64 baselink_to_tag_transform_, FrameTransform frameTransform
            , double width, double height) {
        return detectedInImage_2(robotId_, imageCaptureTime_, robotPositionInImage_, baselink_to_tag_transform_, frameTransform
                , width, height);
    }

    public Se3_F64 detectedInImage_2(String robotId_, java.util.Date imageCaptureTime_, PixelPosition robotPositionInImage_, Se3_F64 baselink_to_tag_transform_, FrameTransform frameTransform
            , double width, double height) {
        System.out.println("detectedInImage_2("+robotId_+", "+imageCaptureTime_+", "+robotPositionInImage_+", "+baselink_to_tag_transform_+");");
        if(calibrated == state) { System.out.println("detectedInImage_2: ALREADY CALIBRATED"); }
        TEMP_NUM_OBS_EQUAL_NUM_PLANNED = 10;
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
        Se3_F64 estimatedCameraPose = estimateExtrinsics(width, height);
        if(null != estimatedCameraPose){
            state = calibrated;
            System.out.println("SCEC: detectedInImage_2: estimateExtrinsics()!!");
            return estimatedCameraPose;
        } else {
            System.out.println("SCEC: detectedInImage_2: NOT estimateExtrinsics() yet.");
            return null;
        }
    }

    private static final double PIXEL_DIFFERENCE_THRESHOLD = 2.0d;  // if closer than 2px in x AND y, will reject as too similar - NOTE; assumes camera is fixed pose

    private void recordRobotDetectionInImage(Observation2 detectionInImage) {
        synchronized (associatedData_LOCK) {
            for(Observation2 detection: detectionInImages) {
                if(     Math.abs(detection.pixelPosition.getU()-detectionInImage.pixelPosition.getU()) < PIXEL_DIFFERENCE_THRESHOLD
                        &&
                        Math.abs(detection.pixelPosition.getV()-detectionInImage.pixelPosition.getV()) < PIXEL_DIFFERENCE_THRESHOLD
                        ) {
                    System.out.println("recordRobotDetectionInImage: rejected: "
                            +"  u="+detection.pixelPosition.getU()+", u="+detectionInImage.pixelPosition.getU()
                            +", v="+detection.pixelPosition.getV()+", v="+detectionInImage.pixelPosition.getV()
                            +": "+detectionInImage+" =~= "+detection);
                    return;
                }
            }
            detectionInImages.add(detectionInImage);
        }
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
            synchronized (associatedData_LOCK) {
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

    public FrameTransform askRobotForItsPoseFrame(java.util.Date date) {                          System.out.println("SCEC: askRobotForItsPose: "+stateString());
//        FrameTransform transform = robotPoseMeasure.askRobotForPoseFrameAsync(date);       System.out.println("SCEC: askRobotForItsPose: transform ="+transform);
//        return transform;
        return null;
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
        synchronized (observations_LOCK) {
            observations = new ArrayList<>();  //        robotGoalPoses = new ArrayList<>(1);
        }
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



    private int TEMP_NUM_OBS_EQUAL_NUM_PLANNED = 20;


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

    public void listObservations() {
        System.out.println("listObservations(): start");
        synchronized (observations_LOCK) {
            System.out.println("listObservations(): in synchronized block: observations.size()=" + observations.size());
            double[] IMAGE_PIXEL_2D_DATA_POINTS__ = new double[observations.size() * 2];
            double[] WORLD_3D_DATA_POINTS__ = new double[observations.size() * 3];

            int i_ = 0;
            for (Observation2 obs : observations) {
                IMAGE_PIXEL_2D_DATA_POINTS__[i_ + 0] = obs.pixelPosition.getU();
                IMAGE_PIXEL_2D_DATA_POINTS__[i_ + 1] = obs.pixelPosition.getV();

                WORLD_3D_DATA_POINTS__[i_ + 0] = obs.map_to_tag_transform_boofcv.getT().getX();
                WORLD_3D_DATA_POINTS__[i_ + 1] = obs.map_to_tag_transform_boofcv.getT().getY();
                WORLD_3D_DATA_POINTS__[i_ + 2] = obs.map_to_tag_transform_boofcv.getT().getZ();

                printToConsole(IMAGE_PIXEL_2D_DATA_POINTS__, WORLD_3D_DATA_POINTS__, i_);
                i_++;
            }
            logObservations2ToFile();
            this.poseEstimator2D3D.estimatePose(observations, this);
        }
    }


    public void logObservations2ToFile() {
        double[] IMAGE_PIXEL_2D_DATA_POINTS__ = new double[observations.size() * 2];
        double[] WORLD_3D_DATA_POINTS__ = new double[observations.size() * 3];
        int i_;
        {
            i_ = 0;
            String observation2DString = "pixels_2D:\n";
            for (Observation2 obs : observations) {
                IMAGE_PIXEL_2D_DATA_POINTS__[i_ + 0] = obs.pixelPosition.getU();
                IMAGE_PIXEL_2D_DATA_POINTS__[i_ + 1] = obs.pixelPosition.getV();
                observation2DString = observation2DString + "\n" + formatObservation2D(IMAGE_PIXEL_2D_DATA_POINTS__, i_);
                i_++;
            }
            fileLogger.printlnToFile(observation2DString, DateAndTime.nowAsDate());
        }
        {
            i_ = 0;
            String observation3DString = "points_3D:\n";
            for (Observation2 obs : observations) {
                WORLD_3D_DATA_POINTS__[i_ + 0] = obs.map_to_tag_transform_boofcv.getT().getX();
                WORLD_3D_DATA_POINTS__[i_ + 1] = obs.map_to_tag_transform_boofcv.getT().getY();
                WORLD_3D_DATA_POINTS__[i_ + 2] = obs.map_to_tag_transform_boofcv.getT().getZ();
                observation3DString = observation3DString + "\n" + formatObservation3D(WORLD_3D_DATA_POINTS__, i_);
                i_++;
            }
            fileLogger.printlnToFile(observation3DString, DateAndTime.nowAsDate());
        }
        {
            i_ = 0;
            String observation2DString = "pixels_2D=[...\n";
            for (Observation2 obs : observations) {
                observation2DString = observation2DString
                        + "\t"  + obs.pixelPosition.getU()
                        + " , " + obs.pixelPosition.getV()
                        + " ;";
                i_++;
            }
            observation2DString = observation2DString + " ]; ";
            fileLogger.printlnToFile(observation2DString, DateAndTime.nowAsDate());
        }
        {
            i_ = 0;
            String observation3DString = "points_3D:\n";
            for (Observation2 obs : observations) {
                observation3DString = observation3DString
                        + "\t"  + obs.map_to_tag_transform_boofcv.getT().getX()
                        + " , " + obs.map_to_tag_transform_boofcv.getT().getY()
                        + " , " + obs.map_to_tag_transform_boofcv.getT().getZ()
                        + " ;";
                i_++;
            }
            observation3DString = observation3DString + " ]; ";
            fileLogger.printlnToFile(observation3DString, DateAndTime.nowAsDate());
        }
    }


    /**
     * Callback for pose estimates: receives the pose estimate from the estimation process (EPnP, etc.).
     * @param poseEstimate the pose estimate from the estimation process (EPnP, etc.).
     */
    public void poseEstimate2D3D(william.chamberlain.androidvosopencvros.device.Pose poseEstimate) {
        String poseEstimate2D3DString = "poseEstimate2D3D(x, y, z, qx, qy, qz, qw) =\n";
        poseEstimate2D3DString = poseEstimate2D3DString
                + poseEstimate.getPosition().getX()
                + poseEstimate.getPosition().getY() + " , "
                + poseEstimate.getPosition().getZ() + " , "
                + poseEstimate.getOrientation().getX() + " , "
                + poseEstimate.getOrientation().getY() + " , "
                + poseEstimate.getOrientation().getZ() + " , "
                + poseEstimate.getOrientation().getW();
        fileLogger.printlnToFile(poseEstimate2D3DString, DateAndTime.nowAsDate());  // could use the datetime the request was made - have to pass through the loop

        String poseEstimate2D3DString2    = "poseEstimate2D3D=\n";
        poseEstimate2D3DString2 = poseEstimate2D3DString2
                + "x=" + poseEstimate.getPosition().getX()
                + "y=" + poseEstimate.getPosition().getY() + " , "
                + "z=" + poseEstimate.getPosition().getZ() + " , "
                + "qx=" + poseEstimate.getOrientation().getX() + " , "
                + "qy=" + poseEstimate.getOrientation().getY() + " , "
                + "qz=" + poseEstimate.getOrientation().getZ() + " , "
                + "qw=" + poseEstimate.getOrientation().getW();
        fileLogger.printlnToFile(poseEstimate2D3DString2, DateAndTime.nowAsDate());  // could use the datetime the request was made - have to pass through the loop

        System.out.println("\n" + poseEstimate2D3DString + "\n" + poseEstimate2D3DString2 + "\n");

        //
        //  public void setPose(double[] poseXyz, double[] orientationQuaternionXyzw_)
        posedEntity.setPose(poseEstimate);
    }

    private String formatObservation2D(double[] IMAGE_PIXEL_2D_DATA_POINTS__, int i_) {
        return "pixels_2D=(" + IMAGE_PIXEL_2D_DATA_POINTS__[i_ + 0] + ", " + IMAGE_PIXEL_2D_DATA_POINTS__[i_ + 1] + ") ";
    }
    private String formatObservation3D(double[] WORLD_3D_DATA_POINTS__, int i_) {
        return "world_3D=(" + WORLD_3D_DATA_POINTS__[i_ + 0] + ", " + WORLD_3D_DATA_POINTS__[i_ + 1] + ", " + WORLD_3D_DATA_POINTS__[i_ + 2] + ")";
    }
    private void printToConsole(double[] IMAGE_PIXEL_2D_DATA_POINTS__, double[] WORLD_3D_DATA_POINTS__, int i_) {
        System.out.println(
                "observations(" + i_ + ") = \n" +
                        "pixels_2D=(" + IMAGE_PIXEL_2D_DATA_POINTS__[i_ + 0] + ", " + IMAGE_PIXEL_2D_DATA_POINTS__[i_ + 1] + ") " +
                        "world_3D=(" + WORLD_3D_DATA_POINTS__[i_ + 0] + ", " + WORLD_3D_DATA_POINTS__[i_ + 1] + ", " + WORLD_3D_DATA_POINTS__[i_ + 2] + ")" );
    }

    private final Object associatedData_LOCK = new Object();

    public Se3_F64 estimateExtrinsicsFromObservations(double imageWidth, double imageHeight) {
        TEMP_NUM_OBS_EQUAL_NUM_PLANNED = 10;
        if(true) {
            return null;
        }
        if(null == observations  || observations.size() < TEMP_NUM_OBS_EQUAL_NUM_PLANNED) {
            return null;
        }
        double[] IMAGE_PIXEL_2D_DATA_POINTS__;
        double[] WORLD_3D_DATA_POINTS__;
        synchronized (observations_LOCK) {
            IMAGE_PIXEL_2D_DATA_POINTS__ = new double[observations.size() * 2];
            WORLD_3D_DATA_POINTS__ = new double[observations.size() * 3];

            int i_ = 0;
            for (Observation2 obs : observations) {
                System.out.println("SCEC: estimateExtrinsicsFromObservations: observation pixels "
                        + obs.pixelPosition.getU() +" , "
                        + obs.pixelPosition.getV());
                i_++;
            }
            i_ = 0;
            for (Observation2 obs : observations) {
                System.out.println("SCEC: estimateExtrinsicsFromObservations: observation robotpose "
                        + obs.map_to_tag_transform_boofcv.getTranslation().getX() +" , "
                        + obs.map_to_tag_transform_boofcv.getTranslation().getY() +" , "
                        + obs.map_to_tag_transform_boofcv.getTranslation().getZ());
                i_++;
            }
            i_ = 0;
            for (Observation2 obs : observations) {
                IMAGE_PIXEL_2D_DATA_POINTS__[i_ + 0] = obs.pixelPosition.getU();
                IMAGE_PIXEL_2D_DATA_POINTS__[i_ + 1] = obs.pixelPosition.getV();

                WORLD_3D_DATA_POINTS__[i_ + 0] = obs.map_to_tag_transform_boofcv.getT().getX();
                WORLD_3D_DATA_POINTS__[i_ + 1] = obs.map_to_tag_transform_boofcv.getT().getY();
                WORLD_3D_DATA_POINTS__[i_ + 2] = obs.map_to_tag_transform_boofcv.getT().getZ();

                printToConsole(IMAGE_PIXEL_2D_DATA_POINTS__, WORLD_3D_DATA_POINTS__, i_);

                i_++;
            }

        }
        opencv_core.Mat transformOpenCVMat = calculateExtrinsics(IMAGE_PIXEL_2D_DATA_POINTS__, WORLD_3D_DATA_POINTS__, imageWidth, imageHeight);

        Se3_F64 transformBoofCVMat = convert_Mat_OpenCV_to_Se3_f64_BoofCV(transformOpenCVMat);

        return transformBoofCVMat;
    }

    public void estimateExtrinsics() {
        System.out.println("SCEC: estimateExtrinsics(): logging observations then estimating the pose.");
        logObservations2ToFile();
        this.poseEstimator2D3D.estimatePose(observations, this);
    }

    /** @return the estimated camera extrinsics (pose: position and orientation) in world coordinates. */
    private Se3_F64 estimateExtrinsics(double width, double height) {
        int i_ = 0;
        /*
            if(null != associatedData  &&  associatedData.size() >= TEMP_NUM_OBS_EQUAL_NUM_PLANNED) {
                System.out.println("SCEC: estimateExtrinsics: TRUE");
                return true; }
         */
        synchronized (associatedData_LOCK) {
            if(null != associatedData  &&  associatedData.size() >= TEMP_NUM_OBS_EQUAL_NUM_PLANNED) {
                double[] IMAGE_PIXEL_2D_DATA_POINTS_ = new double[associatedData.size() * 2];
                double[] WORLD_3D_DATA_POINTS_ = new double[associatedData.size() * 3];
                for (AssociatedData associatedDatapoint : associatedData) {
                    System.out.println("SCEC: estimateExtrinsics: AssociatedData " + i_ + " = " + associatedDatapoint);
                    IMAGE_PIXEL_2D_DATA_POINTS_[i_ + 0] = associatedDatapoint.detectionInImageData.pixelPosition.getU();
                    IMAGE_PIXEL_2D_DATA_POINTS_[i_ + 1] = associatedDatapoint.detectionInImageData.pixelPosition.getV();

                    WORLD_3D_DATA_POINTS_[i_ + 0] = associatedDatapoint.detectionInImageData.map_to_tag_transform_boofcv.getT().getX();
                    WORLD_3D_DATA_POINTS_[i_ + 1] = associatedDatapoint.detectionInImageData.map_to_tag_transform_boofcv.getT().getY();
                    WORLD_3D_DATA_POINTS_[i_ + 2] = associatedDatapoint.detectionInImageData.map_to_tag_transform_boofcv.getT().getZ();

                    i_++;
                }
                opencv_core.Mat transformOpenCVMat = calculateExtrinsics(IMAGE_PIXEL_2D_DATA_POINTS_, WORLD_3D_DATA_POINTS_, width, height);

                Se3_F64 transformBoofCVMat = convert_Mat_OpenCV_to_Se3_f64_BoofCV(transformOpenCVMat);

                System.out.println("SCEC: estimateExtrinsics: TRUE");
                return transformBoofCVMat;
            }
        }
        System.out.println("SCEC: estimateExtrinsics: false");
        return null;
    }

    @NonNull
    private Se3_F64 convert_Mat_OpenCV_to_Se3_f64_BoofCV(opencv_core.Mat transformOpenCVMat) {
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
        return transformBoofCVMat;
    }






    private double[] IMAGE_PIXEL_2D_DATA_POINTS = {
            1538d, 589d,
            1723d, 1013d };

    private double[] WORLD_3D_DATA_POINTS = new double[]{
            622.0000d, 643.5000d, 90.0000d,
            655.0000d, 643.5000d, 0.000d };


    public opencv_core.Mat calculateExtrinsics(double[] IMAGE_PIXEL_2D_DATA_POINTS_, double[] WORLD_3D_DATA_POINTS_, double imageWidth, double imageHeight) {
            System.out.println("calculateExtrinsics: before opencv_calib3d.solvePnPRansac");
        opencv_core.Mat objectPoints   = new opencv_core.Mat(WORLD_3D_DATA_POINTS_.length/3, 3, CV_64FC1); // 10 measurements, 3 dimensions, 1 channel: could also have Mat(10,1,3)
        objectPoints.getDoubleBuffer().put(WORLD_3D_DATA_POINTS_);
            System.out.println("calculateExtrinsics: objectPoints = "+matToString(objectPoints));
            System.out.println("calculateExtrinsics: objectPoints.toString()="+objectPoints.toString());
            System.out.println("calculateExtrinsics: objectPoints.size().toString()="+objectPoints.size().toString());
            // i.e. a 10-element vector with each element a 3d measurement/point :
            // Nx3 1-channel or 1xN/Nx1 3-channel
            // HeadPose example uses coordinates based on the head model centre - relative to camera focal axis - [ x=right , y=up , z=depth/distance=toward-camera]
        List<opencv_core.Point2f> imagePointVector = new ArrayList<>();
        double[] pixeldata = IMAGE_PIXEL_2D_DATA_POINTS_;
        opencv_core.Mat imagePoints    = new opencv_core.Mat(WORLD_3D_DATA_POINTS_.length/3, 2, CV_64FC1);
        imagePoints.getDoubleBuffer().put(pixeldata);
            System.out.println("calculateExtrinsics: imagePoints = "+matToString(imagePoints));
        imagePoints.getDoubleBuffer().toString();
            System.out.println("calculateExtrinsics: imagePoints.toString()="+imagePoints.toString());
            System.out.println("calculateExtrinsics: imagePoints.size().toString()="+imagePoints.size().toString());
        imagePoints.size().toString();
            //            org.bytedeco.javacpp.Indexer indexer = imagePoints.createIndexer();
            //            indexer.put(row,col,val)
            //            Mat imagePoints    = new Mat(10, 2, CV_64FC1); // pixel coordinates, as [ u=x=right, v=y=down ]
            //            imagePoints.push_back(new Mat(opencv_core.Point2f(1538.0f,589.0f)));

        opencv_core.Mat rVec_Estimated          = new opencv_core.Mat(3, 3, CV_64FC1); // Output rotation   _vector_ : world-to-camera
        opencv_core.Mat tVec_Estimated          = new opencv_core.Mat(3, 1, CV_64FC1); // Output translation vector  : world-to-camera
        opencv_core.Mat cameraIntrinsicsMatrix  = new opencv_core.Mat(3, 3, CV_64FC1);
        opencv_core.Mat distCoeffs              = new opencv_core.Mat(5, 1, CV_64FC1 );

//        see /mnt/nixbig/ownCloud/project_AA1__2_extrinsics_calibration/project_AA1_2_extrinsics__phone_data_recording/VOS_data_2018_01_21_16_51_11_calibration_pattern_Galaxy3_home/intrinsics_matlab_3radial_2tangental_0skew.txt
//        Radial
//                [0.004180016841640, 0.136452931271259, -0.638647134308425]
//        Tangental
//                //[-0.001666231998527, -8.160213039217031e-05]
//                [-0.001666231998527, -0.00008160213039217031]
//        Focal length
//                //[3.229596901156589e+02, 3.238523693059909e+02]
//                [322.9596901156589, 323.8523693059909]
//        Principal point
//                //[1.768267919600727e+02, 1.467681514313797e+02]
//                [176.8267919600727, 146.7681514313797]
        double[] camera_intrinsics_opencv = {
                322.9596901156589d,  000.0000000000000d,  176.8267919600727d,
                000.0000000000000d,  323.8523693059909d,  146.7681514313797d,
                000.0000000000000d,  000.0000000000000d,  001.0000000000000d };
//                FOCAL_LENGTH_X,               0.0d,  PRINCIPAL_POINT_X,
//                          0.0d,     FOCAL_LENGTH_Y,  PRINCIPAL_POINT_Y,
//                          0.0d,               0.0d,               1.0d};
        cameraIntrinsicsMatrix.getDoubleBuffer().put(camera_intrinsics_opencv);
            System.out.println("calculateExtrinsics: cameraIntrinsicsMatrix = \n"+matToString(cameraIntrinsicsMatrix));
        distCoeffs.getDoubleBuffer().put(new double[]{
                 0.004180016841640d,  0.136452931271259d, /* radial coefficients 1 and 2 */
                -0.001666231998527d, -0.00008160213039217031d,                             /* tangental coefficients 1 and 2 */
                -0.638647134308425d}    /*radial coefficient 3*/ );
//                RADIAL_1,    RADIAL_2, /* radial coefficients 1 and 2 */
//                TANGENTAL_1, TANGENTAL_2,                             /* tangental coefficients 1 and 2 */
//                RADIAL_3}    /*radial coefficient 3*/ );
            System.out.println("calculateExtrinsics: distCoeffs = \n"+matToString(distCoeffs));

        //<editor-fold desc="Details and links for OpenCV / JavaCV matrices and solvePnP">
        /*-----------
                @param cameraIntrinsicsMatrix Input camera matrix \f$A = \vecthreethree
                      {fx}{0}{cx}
                      {0}{fy}{cy}
                      {0}{0}{1}\f$ .
                @param distCoeffs Input vector of distortion coefficients
                     \f$( k_1, k_2, p_1, p_2  [, k_3  [, k_4, k_5, k_6  [, s_1, s_2, s_3, s_4  [, \tau_x, \tau_y] ] ] ] )\f$ of
                4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are
                assumed.
                @param rvec Output rotation vector (see Rodrigues ) that, together with tvec , brings points from
                the model coordinate system to the camera coordinate system.
                @param tvec Output translation vector.
                @Namespace("cv") public static native @Cast("bool") boolean solvePnPRansac( @ByVal Mat objectPoints, @ByVal Mat imagePoints,
                                          @ByVal Mat cameraIntrinsicsMatrix, @ByVal Mat distCoeffs,
                                          @ByVal Mat rvec, @ByVal Mat tvec,
                                          @Cast("bool") boolean useExtrinsicGuess/=false/, int iterationsCount/=100/,
                                          float reprojectionError/*=8.0/, double confidence/*=0.99/,
                                          @ByVal(nullValue = "cv::OutputArray(cv::noArray())") Mat inliers, int flags/*=cv::SOLVEPNP_ITERATIVE/ );
        opencv_calib3d.solvePnPRansac(objectPoints, imagePoints,
                cameraIntrinsicsMatrix,   distCoeffs,
                rVec_Estimated, tVec_Estimated,
                false,          100,
                0.8f,           0.99d);

          http://answers.opencv.org/question/87546/solvepnp-fails-with-perfect-coordinates-and-cvposit-passes/
        From my knowledge, the pose estimation problem (or PnP problem) is a non linear problem. Thus, it is a non trivial problem and it is always possible to converge to a local minima I think. That's why there are plenty of methods to estimate the camera pose:
            Model-Based Object Pose in 25 Lines of Code (The POSIT method)
            Complete Solution Classification for the Perspective-Three-Point Problem (SOLVEPNP_P3P)
            EPnP: Efficient Perspective-n-Point Camera Pose Estimation (SOLVEPNP_EPNP)
            A Direct Least-Squares (DLS) Method for PnP (SOLVEPNP_DLS)
            Exhaustive Linearization for Robust Camera Pose and Focal Length Estimation (SOLVEPNP_UPNP)
            and plenty of other methods that are not present in OpenCV 3.1.
            ...
            I observed a completly different behavior with and without the cast to float but that is not related to your issue, just that a small "noise" produces very different results for the DLT.

         note: Matlab is good - use Matlab
         note: GTSAM works    - could use that
         https://github.com/opencv/opencv/issues/4854  -  solvePnP: is not handling all degenerated cases
         https://github.com/opencv/opencv/issues/6117  -  solvePnP fails with perfect coordinates (and cvPOSIT passes)
            Read down, and expand the code and the comments:
            "
                With solvePnp_P3p (cv::SOLVEPNP_P3P) the error is about 0.02961 pixels and the order does not matter much
                With solvePnp_Iterative_InitialGuess (cv::SOLVEPNP_ITERATIVE) the error can be 0 pixels if a good initial extrinsic guess is given (otherwise don't hope for any convergence). The order does not matter much with SOLVEPNP_ITERATIVE.
            "
            --> might be good to run solvePnp_P3p -> initial guess -> solvePnp_Iterative_InitialGuess
         http://answers.opencv.org/question/87546/solvepnp-fails-with-perfect-coordinates-and-cvposit-passes/
         see https://github.com/pthom/TestSolvePnp
         opencv_calib3d.solvePnP
         http://answers.opencv.org/question/74327/solvepnp-bad-result-planar-marker/ - SolvePNP Bad Result Planar Marker
           -> https://xuchi.weebly.com/rpnp.html - A Robust O(n) Solution  to the Perspective-n-Point Problem
                 Converting from image coordinate to normalized coordinate
          http://nghiaho.com/?page_id=576 - Pose Estimation For Planar Target -  It is used to determine the pose of a planar target.

        @param cameraMatrix Input camera matrix \f$A = \vecthreethree{fx}{0}{cx}{0}{fy}{cy}{0}{0}{1}\f$ .
        @param distCoeffs Input vector of distortion coefficients
            \f$(k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6 [, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\f$ of
            4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are
            assumed.
            public static final double[] CAMERA_DISTORTION_COEFFICIENTS_5_OPENCV = {
            0.007669604014085d, 0.083222884846111d,
            0.0d, 0.0d,
            -0.412608486749279d};
         -----------*/
        //</editor-fold>
        opencv_core.Mat inliers = null;
        opencv_calib3d.solvePnPRansac(
                objectPoints, imagePoints,
                cameraIntrinsicsMatrix, distCoeffs,
                rVec_Estimated, tVec_Estimated);
        opencv_core.Mat cameraPoseInWorldCoords = decomposeToCameraPose(rVec_Estimated, tVec_Estimated, "none_ransac");

        opencv_calib3d.solvePnPRansac(
                objectPoints, imagePoints,
                cameraIntrinsicsMatrix, distCoeffs,
                rVec_Estimated, tVec_Estimated,
                false, 1000,
                8.0f, 0.99d,
                inliers, org.bytedeco.javacpp.opencv_calib3d.SOLVEPNP_P3P);
        cameraPoseInWorldCoords = decomposeToCameraPose(rVec_Estimated, tVec_Estimated, "P3P_ransac");

        opencv_calib3d.solvePnPRansac(
                objectPoints, imagePoints,
                cameraIntrinsicsMatrix, distCoeffs,
                rVec_Estimated, tVec_Estimated,
                false, 1000,
                8.0f, 0.99d,
                inliers, opencv_calib3d.SOLVEPNP_EPNP);
        cameraPoseInWorldCoords = decomposeToCameraPose(rVec_Estimated, tVec_Estimated, "Eff_PNP_ransac");

        opencv_calib3d.solvePnPRansac(
                objectPoints, imagePoints,
                cameraIntrinsicsMatrix, distCoeffs,
                rVec_Estimated, tVec_Estimated,
                false, 1000,
                4.0f, 0.99d,
                inliers, org.bytedeco.javacpp.opencv_calib3d.SOLVEPNP_P3P);
        cameraPoseInWorldCoords = decomposeToCameraPose(rVec_Estimated, tVec_Estimated, "P3P_ransac");

        opencv_calib3d.solvePnPRansac(
                objectPoints, imagePoints,
                cameraIntrinsicsMatrix, distCoeffs,
                rVec_Estimated, tVec_Estimated,
                false, 1000,
                4.0f, 0.99d,
                inliers, opencv_calib3d.SOLVEPNP_EPNP);
        cameraPoseInWorldCoords = decomposeToCameraPose(rVec_Estimated, tVec_Estimated, "Eff_PNP_ransac");

        opencv_calib3d.solvePnPRansac(
                objectPoints, imagePoints,
                cameraIntrinsicsMatrix, distCoeffs,
                rVec_Estimated, tVec_Estimated,
                false, 1000,
                1.0f, 0.99d,
                inliers, org.bytedeco.javacpp.opencv_calib3d.SOLVEPNP_P3P);
        cameraPoseInWorldCoords = decomposeToCameraPose(rVec_Estimated, tVec_Estimated, "P3P_ransac");

        opencv_calib3d.solvePnPRansac(
                objectPoints, imagePoints,
                cameraIntrinsicsMatrix, distCoeffs,
                rVec_Estimated, tVec_Estimated,
                false, 1000,
                1.0f, 0.99d,
                inliers, opencv_calib3d.SOLVEPNP_EPNP);
        cameraPoseInWorldCoords = decomposeToCameraPose(rVec_Estimated, tVec_Estimated, "Eff_PNP_ransac");

        opencv_calib3d.solvePnP(objectPoints, imagePoints, cameraIntrinsicsMatrix, distCoeffs, rVec_Estimated, tVec_Estimated
                ,false, opencv_calib3d.SOLVEPNP_EPNP );
        cameraPoseInWorldCoords = decomposeToCameraPose(rVec_Estimated, tVec_Estimated, "Eff_PNP_plain");
        return cameraPoseInWorldCoords;
    }

    @NonNull
    private opencv_core.Mat decomposeToCameraPose(opencv_core.Mat rVec_Estimated, opencv_core.Mat tVec_Estimated, String method) {
            System.out.println("calc ext: "+method+":  tVec_Estimated = "+matToString(tVec_Estimated));
            System.out.println("calc ext: "+method+":  rVec_Estimated = "+matToString(rVec_Estimated));
        opencv_core.Mat rotationMatrix = new opencv_core.Mat(3, 3, CV_64FC1);
        opencv_calib3d.Rodrigues(rVec_Estimated,rotationMatrix);
            System.out.println("calc ext: "+method+":  rotationMatrix = \n"+matToString(rotationMatrix));

        opencv_core.MatExpr rotationMatrixTransposeTemp = rotationMatrix.t();
        opencv_core.Mat rotationMatrixTranspose = new opencv_core.Mat(1,1,CV_64FC1);
        rotationMatrixTranspose.put(rotationMatrixTransposeTemp);
            System.out.println("calc ext: "+method+":  rotationMatrixTranspose = \n"+matToString(rotationMatrixTranspose));

            //        opencv_core.Mat negOne = new opencv_core.Mat(3,3,CV_64FC1);
            //        setIdentity(negOne, new opencv_core.Scalar(-1.0d));
            //            System.out.println("calc ext: "+method+":  negOne = "+matToString(negOne));
            //        negOne.getDoubleBuffer().put(new double[]{
            //                -1.0d, -1.0d, -1.0d,
            //                -1.0d, -1.0d, -1.0d,
            //                -1.0d, -1.0d, -1.0d });
            //            System.out.println("calc ext: "+method+":  negOne = "+matToString(negOne));

            //            opencv_core.Mat one = new opencv_core.Mat(3,3,CV_64FC1); setIdentity(one, new opencv_core.Scalar(1.0d));
            //            System.out.println("calc ext: "+method+":  one = "+matToString(one));
            //            opencv_core.MatExpr translationInverseTemp_One = one.mul(rotationMatrixTranspose);
            //            System.out.println("calc ext: "+method+":  translationInverseTemp_One.asMat() = "+matToString(translationInverseTemp_One.asMat()));
            //            translationInverseTemp_One = opencv_core.multiply(rotationMatrixTranspose,one);
            //            System.out.println("calc ext: "+method+":  translationInverseTemp_One.asMat() = "+matToString(translationInverseTemp_One.asMat()));

        opencv_core.MatExpr rotationMatrixTransposeNegated = opencv_core.multiply(rotationMatrixTranspose, -1.0); // element-wise
            System.out.println("calc ext: "+method+":  rotationMatrixTransposeNegated.asMat() = \n"+matToString(rotationMatrixTransposeNegated.asMat()));

        opencv_core.Mat translationInverseTempTemp =new opencv_core.Mat(3,3,CV_64FC1);
            //gemm( Mat src1,  Mat src2,
            // double alpha,
            // opencv_core.Mat src3, double beta,
            // opencv_core.Mat dst,
            // int flags/*=0*/);  last argument is sum of flags: each of GEMM_1_T, GEMM_2_T , GEMM_3_T means to transpose src1, src2, src3 before doing the multiplication: default is 0 = no transposing
            //  https://stackoverflow.com/questions/10168058/basic-matrix-multiplication-in-opencv-for-android
            //  gemm(src1, src2, alpha, src3, beta, dest, flags)
            //  dest = alpha * src1 * src2 + beta * src3
            //    last argument is sum of flags: each of GEMM_1_T, GEMM_2_T , GEMM_3_T means to transpose src1, src2, src3 before doing the multiplication: default is 0 = no transposing
            //    gemm(src1, src2, alpha, src3, beta, dst, GEMM_1_T + GEMM_3_T)   -->  dst = alpha*src1.t()*src2 + beta*src3.t();
        opencv_core.gemm(
            rotationMatrixTransposeNegated.asMat(), tVec_Estimated,     // src1, src2,
            1.0d,                                                           // alpha
            opencv_core.Mat.zeros(3, 3, CV_64FC1).asMat(), 0.0d,            // src3, beta
            translationInverseTempTemp,                                     // dest
            0 );                                                            // flags = 0 = default = no transposing

            System.out.println("calc ext: "+method+":  rotationMatrixTranspose negated mult tVec_Estimated translationInverseTempTemp.asMat() = "+matToString(translationInverseTempTemp));
        opencv_core.Mat translationInverse = new opencv_core.Mat(1,1,CV_64FC1);
        translationInverse.put(translationInverseTempTemp);
            System.out.println("calc ext: "+method+":  translationInverse = \n"+matToString(translationInverse));

        opencv_core.Mat cameraPoseInWorldCoords = new opencv_core.Mat(4, 4, CV_64FC1);
        // As in Python, start is an inclusive left boundary of the range and end is an exclusive right boundary of the range. Such a half-opened interval ...
        // 0..2, 0..2 inclusive
//            Mat camPoseTopLeft = cameraPoseInWorldCoords.apply(new opencv_core.Range(0,3), new opencv_core.Range(0,3));
//            System.out.println("calc ext: "+method+":  camPoseTopLeft before = "+matToString(camPoseTopLeft));
//            camPoseTopLeft.getDoubleBuffer().put(rotationMatrixTranspose.getDoubleBuffer());
//            System.out.println("calc ext: "+method+":  camPoseTopLeft = "+matToString(camPoseTopLeft));
//            System.out.println("calc ext: "+method+":  cameraPoseInWorldCoords = "+matToString(cameraPoseInWorldCoords));  // goes sequentially, not in the shape of top left - how is this useful ??
        cameraPoseInWorldCoords.getDoubleBuffer().put(0, rotationMatrixTranspose.getDoubleBuffer().get(0)); // row one, all cols
        cameraPoseInWorldCoords.getDoubleBuffer().put(1, rotationMatrixTranspose.getDoubleBuffer().get(1)); // row one, all cols
        cameraPoseInWorldCoords.getDoubleBuffer().put(2, rotationMatrixTranspose.getDoubleBuffer().get(2)); // row one, all cols
        cameraPoseInWorldCoords.getDoubleBuffer().put(4, rotationMatrixTranspose.getDoubleBuffer().get(3));
        cameraPoseInWorldCoords.getDoubleBuffer().put(5, rotationMatrixTranspose.getDoubleBuffer().get(4));
        cameraPoseInWorldCoords.getDoubleBuffer().put(6, rotationMatrixTranspose.getDoubleBuffer().get(5));
        cameraPoseInWorldCoords.getDoubleBuffer().put(8, rotationMatrixTranspose.getDoubleBuffer().get(6));
        cameraPoseInWorldCoords.getDoubleBuffer().put(9, rotationMatrixTranspose.getDoubleBuffer().get(7));
        cameraPoseInWorldCoords.getDoubleBuffer().put(10, rotationMatrixTranspose.getDoubleBuffer().get(8));
            System.out.println("calc ext: "+method+":  cameraPoseInWorldCoords after inserting rotation = \n"+matToString(cameraPoseInWorldCoords));

        // 0..2,3..3 inclusive
//            Mat camPoseRightCol = cameraPoseInWorldCoords.apply(new opencv_core.Range(0,3), new opencv_core.Range(3,4));
//            System.out.println("calc ext: "+method+":  camPoseRightCol before = "+matToString(camPoseRightCol));
//            camPoseRightCol.getDoubleBuffer().put(translationInverse.getDoubleBuffer());
//            System.out.println("calc ext: "+method+":  camPoseRightCol = "+matToString(camPoseRightCol));
//            System.out.println("calc ext: "+method+":  cameraPoseInWorldCoords = "+matToString(cameraPoseInWorldCoords));  // goes sequentially, not in the shape of top left - how is this useful ??
        cameraPoseInWorldCoords.getDoubleBuffer().put(3, translationInverse.getDoubleBuffer().get(0));
        cameraPoseInWorldCoords.getDoubleBuffer().put(7, translationInverse.getDoubleBuffer().get(1));
        cameraPoseInWorldCoords.getDoubleBuffer().put(11, translationInverse.getDoubleBuffer().get(2));
            System.out.println("calc ext: "+method+":  cameraPoseInWorldCoords after inserting translation = \n"+matToString(cameraPoseInWorldCoords));

// from here
        double[] homogeneousPadding = new double[]{0.0d, 0.0d, 0.0d, 1.0d};
//                // 3..3,0..3 inclusive
//            cameraPoseInWorldCoords.apply(new opencv_core.Range(3,4), new opencv_core.Range(0,4)).put(new Mat(homogeneousPadding));
//            System.out.println("calc ext: "+method+":  cameraPoseInWorldCoords = "+matToString(cameraPoseInWorldCoords));
        cameraPoseInWorldCoords.getDoubleBuffer().put(12, homogeneousPadding[0]);
        cameraPoseInWorldCoords.getDoubleBuffer().put(13, homogeneousPadding[1]);
        cameraPoseInWorldCoords.getDoubleBuffer().put(14, homogeneousPadding[2]);
        cameraPoseInWorldCoords.getDoubleBuffer().put(15, homogeneousPadding[3]);
            System.out.println("calc ext: "+method+":  cameraPoseInWorldCoords after padding = \n"+matToString(cameraPoseInWorldCoords));

            System.out.println("calc ext: "+method+":  after opencv_calib3d.solvePnPRansac");
        return cameraPoseInWorldCoords;
    }


    private void cameraPoseViaHomography() {
        Mat worldPoints = null;
        Mat imagePoints = null;
        Mat homography = org.opencv.imgproc.Imgproc.getPerspectiveTransform(worldPoints, imagePoints);
    }

    public void setFileLogger(FileLogger fileLogger_) {
        this.fileLogger = fileLogger_;
    }


    public void setPoseEstimator2D3D(PoseEstimator2D3D poseEstimator2D3D_) {
        this.poseEstimator2D3D = poseEstimator2D3D_;
    }

    public void setPosedEntity(PosedEntity posedEntity_) {
        this.posedEntity = posedEntity_;
    }
}


enum PlanningStrategy { fixedSetOfTwo, fixedSet, carryOn, grid }
