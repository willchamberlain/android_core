package william.chamberlain.androidvosopencvros;


import android.os.SystemClock;

import org.apache.commons.httpclient.Header;
import org.apache.commons.logging.Log;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava_geometry.FrameTransform;
import org.ros.rosjava_geometry.FrameTransformTree;
import org.ros.rosjava_geometry.Transform;

import java.util.ArrayList;
import java.util.List;

import actionlib_msgs.GoalStatus;
import actionlib_msgs.GoalStatusArray;
import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
import geometry_msgs.PoseWithCovarianceStamped;
import geometry_msgs.Quaternion;
import vos_aa1.WhereIsAsPub;
import william.chamberlain.androidvosopencvros.ros_types.RosTypes;

import static actionlib_msgs.GoalStatus.ACTIVE;
import static actionlib_msgs.GoalStatus.PENDING;
import static actionlib_msgs.GoalStatus.SUCCEEDED;
import static william.chamberlain.androidvosopencvros.ros_types.RosTypes.header;

/**
 * Created by will on 1/09/17.
 */

public class VosTaskAssignmentSubscriberNode extends AbstractNodeMain implements RobotStatusMonitor, RobotPoseMeasure, RobotGoalPublisher, RobotPoseMonitor {

    private String nodeNamespace = null;
    private VisionSource_WhereIs visionSource_WhereIs = null;
    private Subscriber<WhereIsAsPub> subscriber = null;


    private SmartCameraTopLevelController smartCameraTopLevelController;
    Publisher<PoseStamped> robot_goal_publisher = null;
    private Subscriber<GoalStatusArray> robot_move_base_status_subscriber = null;        // SEE detect_feature_server_3.py  handle_move_base_status()
    private List<Subscriber<PoseWithCovarianceStamped>> robot_pose_covar_subscriber_List = new ArrayList<>();

    static final byte MOVE_BASE_STATUS_INIT = -1;
    byte move_base_status = MOVE_BASE_STATUS_INIT;



    private ConnectedNode connectedNode;

//    TfListener tfListener

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("androidvosopencvros/vos_task_assignment_subscriber");
    }




    public void setNodeNamespace(String nodeNamespace) {
        this.nodeNamespace = nodeNamespace;
    }

    public void setVisionSource_WhereIs(VisionSource_WhereIs visionSource_WhereIs_) {
        this.visionSource_WhereIs = visionSource_WhereIs_;
    }

    public void setSmartCameraTopLevelController(SmartCameraTopLevelController controller_) {
        this.smartCameraTopLevelController = controller_;
    }


    /*** TF/TF2 **************************************/
    // copied from /mnt/nixbig/downloads/chulcher_ros_android_will_fork/android_core/android_15/src/org/ros/android/view/visualization/VisualizationView.java
    //  https://answers.ros.org/question/250292/in-rosjavahow-can-i-transform-the-odom-frame-to-map-frame/
    //  https://answers.ros.org/question/249861/how-can-i-record-the-robot-trajectory-and-show-it-on-the-android-device/

    private final Object mutex = new Object();
    private final FrameTransformTree frameTransformTree = new FrameTransformTree();

    private void startTransformListener() {
        final Subscriber<tf2_msgs.TFMessage> tfSubscriber =
                connectedNode.newSubscriber("tf", tf2_msgs.TFMessage._TYPE);
        tfSubscriber.addMessageListener(new MessageListener<tf2_msgs.TFMessage>() {
            @Override
            public void onNewMessage(tf2_msgs.TFMessage message) {
                synchronized (mutex) {
                    for (geometry_msgs.TransformStamped transform : message.getTransforms()) {
                        frameTransformTree.update(transform);
                    }
                }
            }
        });
        final Subscriber<tf2_msgs.TFMessage> tfStaticSubscriber =
                connectedNode.newSubscriber("tf_static", tf2_msgs.TFMessage._TYPE);
        tfStaticSubscriber.addMessageListener(new MessageListener<tf2_msgs.TFMessage>() {
            @Override
            public void onNewMessage(tf2_msgs.TFMessage message) {
                synchronized (mutex) {
                    for (geometry_msgs.TransformStamped transform : message.getTransforms()) {
                        frameTransformTree.update(transform);
                    }
                }
            }
        });
    }



    @Override
    public void onStart(ConnectedNode connectedNode_) {
        final Log log = connectedNode_.getLog();
        this.connectedNode = connectedNode_;

        startTransformListener();  // start listening to TF :  TODO : move this to a central/singleton TF node :- only want to spend CPU on one at a time

        subscriber = connectedNode_.newSubscriber(nodeNamespace+"/vos_task_assignment_subscriber", WhereIsAsPub._TYPE);
        subscriber.addMessageListener(new MessageListener<WhereIsAsPub>() {                                 // declare inline
            @Override
            public void onNewMessage(WhereIsAsPub message) {
                log.info("message received: \"" + message.getRequestId() + "\" | \""+message.getAlgorithm()+"\" | \""+message.getDescriptor()+"\" | \"" + message.getReturnUrl() + "\"");
                visionSource_WhereIs.dealWithRequestForInformation(message);
            }
        });


        robot_goal_publisher = connectedNode.newPublisher("/move_base_simple/goal", PoseStamped._TYPE);
        robot_goal_publisher.setLatchMode(true);

        /*** Behaviour ?  Input to/from a task/behaviour : TODO: move to a UncalibratedBehaviour instance? TODO: instance per-robot-that-has-task(s) ************************************/
        robot_move_base_status_subscriber = connectedNode_.newSubscriber("/move_base/status",GoalStatusArray._TYPE);     // TODO: hardcoding
        robot_move_base_status_subscriber.addMessageListener(new MessageListener<GoalStatusArray>() {       // declare inline
            @Override
            public void onNewMessage(GoalStatusArray goalStatusArray) {
                if (null == goalStatusArray.getStatusList() || goalStatusArray.getStatusList().size() <= 0) {

                } else {
                    Time rosTime = goalStatusArray.getHeader().getStamp();
                    java.util.Date statusTime = Date.toDate(rosTime);
                    List<GoalStatus> statusList = new ArrayList<GoalStatus>();
                    for (GoalStatus move_base_goalStatus : goalStatusArray.getStatusList()) {
                        if(move_base_goalStatus.getGoalId().getId().startsWith("/move_base")) {                         // TODO: hardcoding
                            statusList.add(move_base_goalStatus);
                            for (RobotStatusChangeListener changeListener : robotStatusChangeListeners) {
                                changeListener.robotStatusChange(statusTime,statusList);
                            }
                        }
                    }
                }
            }
        } );

        String robot_graph_prefix = "";
        addSubscriptionToRobotPose(connectedNode_, robot_graph_prefix);
    }

    private void addSubscriptionToRobotPose(ConnectedNode connectedNode_, String robot_graph_prefix) {
        Subscriber<PoseWithCovarianceStamped> robot_pose_covar_subscriber = null;
        robot_pose_covar_subscriber = connectedNode_.newSubscriber(robot_graph_prefix+"/amcl_pose", PoseWithCovarianceStamped._TYPE);     // TODO: hardcoding
        robot_pose_covar_subscriber.addMessageListener(new PoseListener(robot_graph_prefix));
        robot_pose_covar_subscriber_List.add(robot_pose_covar_subscriber);
    }

    class PoseListener implements MessageListener<PoseWithCovarianceStamped>{
        String robotId=null;
        public PoseListener(String robot_graph_prefix) {
            robotId = robot_graph_prefix;
        }
        @Override
        public void onNewMessage(PoseWithCovarianceStamped pose) {
            for (RobotPoseListener poseListener : robotPoseListeners) {
                if(poseListener.robotIds().contains(robotId)) {
                    poseListener.robotPose(robotId, pose);
                }
            }
        }
    }



    /*** RobotPoseMeasure *****************************************/

    public Transform askRobotForPose() {  // see SmartCameraExtrinsicsCalibrator

        //  https://answers.ros.org/question/250292/in-rosjavahow-can-i-transform-the-odom-frame-to-map-frame/
        //  https://answers.ros.org/question/249861/how-can-i-record-the-robot-trajectory-and-show-it-on-the-android-device/
        // obtain current robot pose
        FrameTransform frameTransform = frameTransformTree.transform(GraphName.of("base_link"), GraphName.of("map"));
        Transform robotPoseInMap = frameTransform.getTransform();

        return robotPoseInMap;

        // obtain current robot position in image
        // if not visible, set robot goal back to last position
        // if not have enough points to estimate floor homography, set another goal for the robot
        // estimate floor homography
        // if estimate is bad, set robot goal to gather more data
        // if estimate is good, done
    }


    /*** RobotStatusMonitor *****************************************/

    @Override // RobotStatusMonitor
    public GoalStatus[] robotStatusChange() {
        return new GoalStatus[0];
    }

    List<RobotStatusChangeListener> robotStatusChangeListeners = new ArrayList<>(1);

    @Override // RobotStatusMonitor
    public void addRobotStatusChangeListener(RobotStatusChangeListener listener_) {
        robotStatusChangeListeners.add(listener_);
    }

    @Override // RobotStatusMonitor
    public void removeRobotStatusChangeListener(RobotStatusChangeListener listener_) {
        robotStatusChangeListeners.remove(listener_);
    }

    /*** RobotPoseMonitor *******************************************/

    List<RobotPoseListener> robotPoseListeners = new ArrayList<>(1);

    @Override
    public void addRobotPoseListener(RobotPoseListener listener_) {
        robotPoseListeners.add(listener_);
    }

    @Override
    public void removeRobotPoseListener(RobotPoseListener listener_) {
        robotPoseListeners.remove(listener_);
    }




    /*** RobotGoalPublisher *****************************************/

    @Override // RobotGoalPublisher
    public void sendRobotGoalInWorldFrame(PoseStamped poseStamped_) {
        robot_goal_publisher.publish(poseStamped_);
    }

    @Override // RobotGoalPublisher
    public void sendRobotGoalInWorldFrame(Transform transform_) {
        PoseStamped poseStamped = robot_goal_publisher.newMessage();
        header(poseStamped, "map");
        RosTypes.updatePoseStampedFromTransform(transform_, poseStamped);

        robot_goal_publisher.publish(poseStamped);
    }


    @Override // RobotGoalPublisher
    public void sendRobotGoalInRobotFrame(Transform transform_) {
        PoseStamped poseStamped = robot_goal_publisher.newMessage();
        header(poseStamped, "base_link");
        RosTypes.updatePoseStampedFromTransform(transform_, poseStamped);

        robot_goal_publisher.publish(poseStamped);
    }


}


