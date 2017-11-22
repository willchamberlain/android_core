package william.chamberlain.androidvosopencvros;

import org.ejml.data.DenseMatrix64F;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;
import org.ros.rosjava_geometry.Quaternion;

import java.util.ArrayList;
import java.util.List;

import actionlib_msgs.GoalStatus;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Quaternion_F64;
import william.chamberlain.androidvosopencvros.ros_types.RosTypes;

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
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.PlanningStrategy.carryOn;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.RobotEnterFromImageSide.NaN;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.RobotEnterFromImageSide.left;
import static william.chamberlain.androidvosopencvros.SmartCameraExtrinsicsCalibrator.RobotEnterFromImageSide.right;
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

    private class Observation {
        PixelPosition pixelPosition;
        Transform     map_to_baselink_pose;
        Se3_F64       baselink_to_tag_transform;
        Se3_F64       map_to_tag_transform_boofcv;
        private Observation(PixelPosition pixelPosition_, Transform map_to_baselink_pose_, Se3_F64 baselink_to_tag_transform_) {
            this.pixelPosition=pixelPosition_;
            this.map_to_baselink_pose=map_to_baselink_pose_;
            this.baselink_to_tag_transform=baselink_to_tag_transform_;

            Se3_F64 mtbp_transform_boofcv = new Se3_F64();
            mtbp_transform_boofcv.setTranslation(
                    map_to_baselink_pose.getTranslation().getX(), map_to_baselink_pose.getTranslation().getY(),
                    map_to_baselink_pose.getTranslation().getZ());
            Quaternion_F64 mtbp_quat_boofcv = new Quaternion_F64(
                    map_to_baselink_pose.getRotationAndScale().getW(), map_to_baselink_pose.getRotationAndScale().getX(),
                    map_to_baselink_pose.getRotationAndScale().getY(), map_to_baselink_pose.getRotationAndScale().getZ());
            DenseMatrix64F mtbp_rotmat_boofcv = new DenseMatrix64F(3, 3);
            mtbp_transform_boofcv.setRotation(ConvertRotation3D_F64.quaternionToMatrix(mtbp_quat_boofcv, mtbp_rotmat_boofcv));

            // NOTE: boofcv does this in reverse order from normal matrix mult notation
            this.map_to_tag_transform_boofcv = new Se3_F64();
            this.baselink_to_tag_transform.concat(mtbp_transform_boofcv,this.map_to_tag_transform_boofcv);
            System.out.println("new Observation:\\n\\t baselink_to_tag_transform="+baselink_to_tag_transform+",\\n\\tmap_to_tag_transform_boofcv="+map_to_tag_transform_boofcv+",\\n\\t "+toString());

            // to get the world point observed, convert this.pose into BoofCV transform matrix via quaternion and translation
            //   transformOfFeatureInVisualModel.setTranslation(position.getX(), position.getY(), position.getZ());
            //   Quaternion_F64(double w, double x, double y, double z) / Quaternion_F64 rotationOfFeatureInVisualModel_q = convertRosToBoofcvQuaternion(visionTask);
            //   transformOfFeatureInVisualModel.setRotation(
            //          ConvertRotation3D_F64.quaternionToMatrix(rotationOfFeatureInVisualModel_q, rotationOfFeatureInVisualModel_m));
            //
        }

        @Override
        public String toString() {
            return "Observation{" +
                    "pixelPosition=" + pixelPosition +
                    ", map_to_baselink_pose=" + map_to_baselink_pose +
                    ", baselink_to_tag_transform=" + baselink_to_tag_transform +
                    ", map_to_tag_transform_boofcv=" + map_to_tag_transform_boofcv +
                    '}';
        }
    }


    enum SelfState {
        uncalibrated, seenRobot, robotMoving, waitingForObs, timeout_waitingForObs, recordObs, calibrated;
    }
//    enum RobotState {
//        unknown,robotMoving, robotStopped;
//    }


    private List<Observation> observations = new ArrayList<>(4);
    private List<Transform> robotGoalPoses = new ArrayList<>(1);
    private SelfState state = uncalibrated;
//    private RobotState robotState = unknown;


    private RobotGoalPublisher robotGoalPublisher;

    private int waitCount = 0;

    private Se3_F64 tag_to_baselink_transform = null;

    // frame tag
    public void robotDetectedInImage(PixelPosition robotPositionInImage, Se3_F64 baselink_to_tag_transform_) {

//        org.ros.rosjava_geometry.Transform transform = org.ros.rosjava_geometry.Transform.identity();
//        transform.multiply()

        tag_to_baselink_transform = baselink_to_tag_transform_;

        System.out.println("SCEC: robotDetectedInImage: start: state == "+state+", robotPositionInImage="+robotPositionInImage);
        if(state == uncalibrated || state == waitingForObs) {
            state = recordObs;
            Transform robotPose = askRobotForItsPose();
            observations.add(new Observation(robotPositionInImage, robotPose, tag_to_baselink_transform));
                    System.out.println("SCEC: robotDetectedInImage: robotPose = "+robotPose);
            if(planRobotPositions()) {
                askRobotToMove();
            } else {
                System.out.println("SCEC: robotDetectedInImage: PLANNING FAILED: state == "+state+", robotPositionInImage="+robotPositionInImage);
            }
            if(estimateExtrinsics()) {
                state = calibrated;
                System.out.println("SCEC: robotDetectedInImage: camera calibrated");
                return;
            }
            state = robotMoving;
        }
        System.out.println("SCEC: robotDetectedInImage: end: state == "+state+", robotPositionInImage="+robotPositionInImage);
    }

    private int robotFinishedMoving_waitCount = 0;

    private final static int FPS_EST         = 5;
    private final static int SECONDS_TO_WAIT = 5;

    public void finishedWithImage() {       //  MainActivity onCameraFrame is basically the timing driver at the moment
        if(state == waitingForObs) {
            if(robotFinishedMoving_waitCount < FPS_EST*SECONDS_TO_WAIT) {
                System.out.println("SCEC: finishedWithImage: robotFinishedMoving_waitCount="+robotFinishedMoving_waitCount+" : waiting ...");
                robotFinishedMoving_waitCount++;
            } else {
                System.out.println("SCEC: finishedWithImage: robotFinishedMoving_waitCount="+robotFinishedMoving_waitCount+" : waited long enough : "+stateString());
                state = timeout_waitingForObs;
                if (planRobotPositions() ) {
                    askRobotToMove();
                    System.out.println("SCEC: finishedWithImage: PLANNING FAILED: robotFinishedMoving_waitCount=" + robotFinishedMoving_waitCount + " : " + stateString());
                } else {
                    robotFinishedMoving_waitCount = 0;
                    System.out.println("SCEC: finishedWithImage: no observations - waiting some more - robotFinishedMoving_waitCount=" + robotFinishedMoving_waitCount + " : " + stateString());
                }
            }
        }
    }



    private RobotPoseMeasure robotPoseMeasure;

    public void setRobotPoseMeasure(RobotPoseMeasure robotPoseMeasure_) {
        this.robotPoseMeasure = robotPoseMeasure_;
    }

    protected Transform askRobotForItsPose() {
        System.out.println("SCEC: askRobotForItsPose: "+stateString());
        Transform transform = robotPoseMeasure.askRobotForPose();
        System.out.println("SCEC: askRobotForItsPose: transform ="+transform);
        return transform;
    }



    /**
     * @return true if the extrinsics have been calculated and can finish the process.
     */
    private boolean estimateExtrinsics() {
        int i_ = 0;
        for(Observation observation : observations) {
            System.out.println("SCEC: estimateExtrinsics: observation "+i_+" = "+observation);
            i_++;
        }
        if(null != observations  &&  observations.size() >= TEMP_NUM_OBS_EQUAL_NUM_PLANNED) {
            System.out.println("SCEC: estimateExtrinsics: TRUE");
            return true;
        }
        System.out.println("SCEC: estimateExtrinsics: false");
        return false;
    }

    enum RobotEnterFromImageSide {
        NaN, left, right
    }

    private RobotEnterFromImageSide robotEnteredFromSide = NaN;

    enum PlanningStrategy {
        fixedSetOfTwo, carryOn, grid
    }

    private int TEMP_NUM_OBS_EQUAL_NUM_PLANNED = -9000;

    private boolean planRobotPositions() {
        if(null != robotGoalPoses && robotGoalPoses.size() > 0) {
            System.out.println("SCEC: planRobotPositions: already have goal poses: "+stateString());
            return true;
        } else {
            if (null==observations || observations.size()<=0) {
                System.out.println("SCEC: planRobotPositions: no observations: cannot calculate pose: "+stateString());
                return false;
            }
            System.out.println("SCEC: planRobotPositions: calculating poses: "+stateString());
//            List<Transform> plannedRobotPositions = new ArrayList<>();
            float scale = 0.25f;
            Observation lastObservation = observations.get(observations.size() - 1);
            System.out.println("SCEC: planRobotPositions: lastObservation="+lastObservation);
            PixelPosition pixelPosition = lastObservation.pixelPosition;
            initialiseSideOfImageRobotEntered(pixelPosition);
            Quaternion robotGoalRotation = RosTypes.copyQuaternion(lastObservation.map_to_baselink_pose);
            Vector3 robotGoalPositionInWorldFrame = RosTypes.copyVector3(lastObservation.map_to_baselink_pose);
            robotGoalPoses = new ArrayList<Transform>(1);

            final PlanningStrategy planningStrategy = carryOn;
            switch(planningStrategy) {
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

    private void initialiseSideOfImageRobotEntered(PixelPosition pixelPosition) {
        if(NaN == robotEnteredFromSide) {
            System.out.println("SCEC: initialiseSideOfImageRobotEntered: robotEnteredFromSide = "+robotEnteredFromSide);
            if (pixelPosition.getU() > pixelPosition.getWidth() / 2.0) {  // right of image
                robotEnteredFromSide = right;
            } else {    // left of image
                robotEnteredFromSide = left;
            }
            System.out.println("SCEC: initialiseSideOfImageRobotEntered: robotEnteredFromSide = "+robotEnteredFromSide);
        }
    }

    private Transform nextPose;
    private final boolean askRobotToMove_worldFrame = true;

    protected void askRobotToMove() {
//        System.out.println("SCEC: askRobotToMove: start: "+stateString());
        state = robotMoving;
        nextPose = robotGoalPoses.remove(0);
        if (askRobotToMove_worldFrame) {
            last_robot_status = -1;
            robotGoalPublisher.sendRobotGoalInWorldFrame(nextPose);
            System.out.println("SCEC: askRobotToMove: sendRobotGoalInWorldFrame("+nextPose+"): "+stateString());
        } else {
            last_robot_status = -1;
            robotGoalPublisher.sendRobotGoalInRobotFrame(nextPose);
            System.out.println("SCEC: askRobotToMove: sendRobotGoalInRobotFrame("+nextPose+"): "+stateString());
        }
        try {
            System.out.println("SCEC: askRobotToMove: start sleep.");
            Thread.sleep(2000);
            System.out.println("SCEC: askRobotToMove: end sleep.");
        } catch(InterruptedException ie) {
            System.out.println("SCEC: askRobotToMove: sleep interrupted: "+ie.getMessage());
        }
//        System.out.println("SCEC: askRobotToMove: end: "+stateString());
    }

    protected void robotIsMoving() {
        System.out.println("SCEC: robotIsMoving: "+stateString());
    }

    private void robotInUncertainMoveState(){
        System.out.println("SCEC: !!: robotInUncertainMoveState: "+stateString());
    }

    protected void robotFinishedMoving() {
        System.out.println("SCEC: robotFinishedMoving: start: "+stateString());
        if(state == robotMoving) {
            state      = waitingForObs;
            System.out.println("SCEC: robotFinishedMoving: set to waitingForObs: "+stateString());
            robotFinishedMoving_waitCount = 0;
        }
        System.out.println("SCEC: robotFinishedMoving: end: "+stateString());
    }

    private byte last_robot_status = PENDING;

    @Override
    public void robotStatusChange(List<GoalStatus> statuses) {
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
        System.out.println("SCEC: uncalibrated: start: "+stateString());
        observations = new ArrayList<>(4);
        robotGoalPoses = new ArrayList<>(1);
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
