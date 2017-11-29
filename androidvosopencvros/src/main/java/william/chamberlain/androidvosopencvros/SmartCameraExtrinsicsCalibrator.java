package william.chamberlain.androidvosopencvros;

import java.util.ArrayList;
import java.util.List;

import actionlib_msgs.GoalStatus;
import geometry_msgs.PoseWithCovarianceStamped;
import georegression.struct.se.Se3_F64;

import org.ros.rosjava_geometry.Transform;

import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.SelfState.calibrated;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.SelfState.uncalibrated;

/**
 * Created by will on 24/11/17.
 */

public class SmartCameraExtrinsicsCalibrator implements RobotStatusChangeListener, RobotPoseListener, VisionTaskRunning {

//    // need an Object to put in the Deque, so wrap an array of Transform in one of these.
//    private class TransformArrayWrapper {
//        Transform[] transformArray = new Transform[1000];
//    }
//    // Log all the data: use a Deque of arrays, because
//    // 1) data comes in in order per source
//    // 2) an array is fast to iterate but low overhead
//    // 3) Deque can remove the oldest array when it is too old, with less overhead than other collections(?)
//    Deque<TransformArrayWrapper> datalog = new ArrayDeque<>();
//
//    public void addRobotPoseToDataLog(Transform transform) {
//
//    }

    ArrayList<DetectionInImageData> detectionInImages = new ArrayList<>(100);

    ArrayList<PoseFromRobotData> robotPoses = new ArrayList<>(100);


    /*****************************************************************/
    // Ask the robot to move, so we can
    // 1) measure the pixel position (2D point)
    // 2) request the robot's pose (3D point)
    // 3) once we have enough 2D-3D correspondences, run solvePnP to estimate the camera pose in the robot's metric map
    private RobotGoalPublisher robotGoalPublisher;



    /*** RobotStatusChangeListener **************************************************************/

    @Override  // RobotStatusChangeListener
    public void robotStatusChange(java.util.Date statusTime, List<GoalStatus> statuses) {

    }

    /*** RobotPoseListener **************************************************************/

    List<String> robotIds = null;

    /**
     * Just to keep everything wrapped up together.
     */
    class PoseFromRobotData {
        String robotId; java.util.Date poseTime; PoseWithCovarianceStamped pose;
        PoseFromRobotData(String robotId_, java.util.Date poseTime_, PoseWithCovarianceStamped pose_) {
            robotId = robotId_; poseTime = poseTime_; pose = pose_;
        }
    }

    @Override  // RobotPoseListener
    public void robotPose(String robotId, PoseWithCovarianceStamped pose) {
        robotPoses.add(new PoseFromRobotData(robotId, Date.toDate(pose.getHeader().getStamp()), pose));
    }

    @Override  // RobotPoseListener
    public List<String> robotIds() {
        if(null == robotIds) { robotIds = new ArrayList<>(1);  robotIds.add(""); }
        return robotIds;
    }

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
        DetectionInImageData detectionInImage = new DetectionInImageData(robotId_, frameTime_, robotPositionInImage_, baselink_to_tag_transform_);
        detectionInImages.add(detectionInImage);
    }

    /** Don't need this yet - records each frame from the camera, for timekeeping/event monitoring. */
    public void imageReceived() {
    }

    /**
     * Just to keep everything wrapped up together
     * :  the 2D point is just the PixelPosition
     * :  baselink_to_tag_transform - the visual model feature pose - is needed later to transform
     *    from the robot's reported base_link pose to the 3D point of the visual feature, so that
     *    the 2D point - 3D point association is for the visual feature
     * :  the date/frameTime is the time that the visual feature/robot was detected, and is used to
     *    associate this data point with the robot's published base_link pose.
     */
    class DetectionInImageData {
        String robotId;  java.util.Date frameTime;  PixelPosition robotPositionInImage;  Se3_F64 baselink_to_tag_transform;

        DetectionInImageData(String robotId_, java.util.Date frameTime_, PixelPosition robotPositionInImage_, Se3_F64 baselink_to_tag_transform_) {
            this.robotId = robotId_; this.frameTime=frameTime_; this.robotPositionInImage=robotPositionInImage_; this.baselink_to_tag_transform=baselink_to_tag_transform_;
        }
    }


    public void associateData(DetectionInImageData detectionInImageData_) {
        long detectionTime = detectionInImageData_.frameTime.getTime();
        long detectionTimeEarly = detectionTime - 100;      // 0.1s
        long detectionTimeLate = detectionTime + 100;       // 0.1s
        long bestDiffMilliseconds = 1000;                   // 1.0s
        int robotPosesSize = robotPoses.size();
        for (int i_=robotPosesSize-1; i_>=0; i_--) {                // most recent into the past ...
            PoseFromRobotData poseFromRobot = robotPoses.get(i_);
            long poseFromRobotTime = poseFromRobot.poseTime.getTime();
            if(detectionTimeLate < poseFromRobotTime) {             // ... so if our upper bound is lower than/before the pose time, ...
                break;                                              // ... we've gone past the window.
            }
            if(detectionTimeEarly <= poseFromRobotTime && detectionTimeLate >= poseFromRobotTime) {  //  pose time is in the bracket for the visual observation time
                long diffMilliseconds = java.lang.Math.abs( detectionTime - poseFromRobotTime );
                if (diffMilliseconds < bestDiffMilliseconds) {

                }
            }
        }
    }


    /*****************************************************************/

    /*
    http://wiki.ros.org/amcl?distro=indigo   :
    amcl_pose (geometry_msgs/PoseWithCovarianceStamped)
    Robot's estimated pose in the map, with covariance.
    */


    private RobotPoseMeasure robotPoseMeasure;

    public void setRobotPoseMeasure(RobotPoseMeasure robotPoseMeasure_) {
        this.robotPoseMeasure = robotPoseMeasure_;
    }

    //  1) time  2) pose  3) covariance/uncertainty
    protected Transform askRobotForItsPose() {
        System.out.println("SCEC: askRobotForItsPose: "+stateString());
        Transform transform = robotPoseMeasure.askRobotForPose();
        System.out.println("SCEC: askRobotForItsPose: transform ="+transform);
        return transform;
    }

    /*****************************************************************/

    public void setRobotGoalPublisher(RobotGoalPublisher robotGoalPublisher) {
        this.robotGoalPublisher = robotGoalPublisher;
    }


    /*****************************************************************/


    enum SelfState {
        uncalibrated, calibrated;
    }

    SelfState state = uncalibrated;

    public String stateString() {
        return "state=" + state;
    }

    public SmartCameraExtrinsicsCalibrator uncalibrated() {
        System.out.println("SCEC: uncalibrated: start: "+stateString());
//        observations = new ArrayList<>(4);
//        robotGoalPoses = new ArrayList<>(1);
        state = uncalibrated;
        System.out.println("SCEC: uncalibrated: end: "+stateString());
        return this;
    }

    public SmartCameraExtrinsicsCalibrator calibrated() {
        System.out.println("SCEC: calibrated: start: "+stateString());
        state = calibrated;
        System.out.println("SCEC: calibrated: end: "+stateString());
        return this;
    }
}
