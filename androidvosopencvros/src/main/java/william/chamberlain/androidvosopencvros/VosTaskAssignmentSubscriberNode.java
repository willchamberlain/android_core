package william.chamberlain.androidvosopencvros;


import org.apache.commons.logging.Log;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava_geometry.FrameTransform;
import org.ros.rosjava_geometry.FrameTransformTree;
import org.ros.rosjava_geometry.Transform;

import java.util.ArrayList;
import java.util.List;

import actionlib_msgs.GoalStatus;
import actionlib_msgs.GoalStatusArray;
import boofcv.alg.segmentation.slic.SegmentSlic;
import geometry_msgs.PoseStamped;
import geometry_msgs.PoseWithCovarianceStamped;
import georegression.struct.se.Se3_F64;
import vos_aa1.GetTf;
import vos_aa1.GetTfRequest;
import vos_aa1.GetTfResponse;
import vos_aa1.Observation2D3D;
import vos_aa1.PoseFrom2D3D;
import vos_aa1.PoseFrom2D3DRequest;
import vos_aa1.PoseFrom2D3DResponse;
import vos_aa1.WhereIsAsPub;
import william.chamberlain.androidvosopencvros.ros_types.RosTypes;

import static william.chamberlain.androidvosopencvros.ros_types.RosTypes.header;

/**
 * Created by will on 1/09/17.
 */

public class VosTaskAssignmentSubscriberNode extends AbstractNodeMain implements RobotStatusMonitor, RobotPoseMeasure, RobotGoalPublisher, RobotPoseMonitor {

    public static final String GET_TF_SERVICE_NAME = "/androidvosopencvros/look_up_transform";
    public static final String ROBOT_GRAPHNAME_PREFIX = "";
    private String nodeNamespace = null;
    private VisionSource_WhereIs visionSource_WhereIs = null;
    private Subscriber<WhereIsAsPub> subscriber = null;


    private SmartCameraTopLevelController smartCameraTopLevelController;
    Publisher<PoseStamped> robot_goal_publisher = null;
    private Subscriber<GoalStatusArray> robot_move_base_status_subscriber = null;        // SEE detect_feature_server_3.py  handle_move_base_status()
    private List<Subscriber<PoseWithCovarianceStamped>> robot_pose_covar_subscriber_List = new ArrayList<>();

    static final byte MOVE_BASE_STATUS_INIT = -1;
    byte move_base_status = MOVE_BASE_STATUS_INIT;

    private ServiceClient<GetTfRequest, GetTfResponse> transformClient;

    private ConnectedNode connectedNode;

    //--- Pose2D3D -----------------------------------

    private ServiceClient<PoseFrom2D3DRequest, PoseFrom2D3DResponse> poseFrom2D3D_serviceClient;
    private PoseFrom2D3DResponseListener poseFrom2D3D_responseListener;

    class PoseFrom2D3DResponseListener implements ServiceResponseListener<PoseFrom2D3DResponse> {
        @Override
        public void onSuccess(PoseFrom2D3DResponse poseFrom2D3DResponse) {
            System.out.println("PoseFrom2D3DResponseListener: onSuccess: start");
            william.chamberlain.androidvosopencvros.device.Pose poseFrom2D3DResponse_myClass = new william.chamberlain.androidvosopencvros.device.Pose(poseFrom2D3DResponse.getPoseEstimate());
            System.out.println("PoseFrom2D3DResponseListener: onSuccess: pose = "+poseFrom2D3DResponse_myClass);
        }

        @Override
        public void onFailure(RemoteException e) {
            System.out.println("PoseFrom2D3DResponseListener: onFailure: "+e.getMessage());
            e.printStackTrace();
            throw new RosRuntimeException(e);
        }
    }


    public void atestMatlabPoseEstimation_kitchen_realPixels() {
        System.out.println("VosTaskAssignmentSubscriberNode: atestMatlabPoseEstimation_kitchen_realPixels: start");
        if(!connected_to_POSE_FROM_2D3D_SERVICE_ROS) {
            System.out.println("WARN: VosTaskAssignmentSubscriberNode: atestMatlabPoseEstimation_kitchen_realPixels: not connected yet: trying to connect now");
            connect_to_POSE_FROM_2D3D_SERVICE_ROS();
        }
        if(null==poseFrom2D3D_serviceClient) {
            System.out.println("WARN: VosTaskAssignmentSubscriberNode: atestMatlabPoseEstimation_kitchen_realPixels: aborting: null==poseFrom2D3D_serviceClient");
            return;
        }
        PoseFrom2D3DRequest request = poseFrom2D3D_serviceClient.newMessage();
//        sensor_msgs.CameraInfo cameraInfo = new CameraInfo();

        sensor_msgs.CameraInfo cameraInfo = connectedNode.getTopicMessageFactory().newFromType(sensor_msgs.CameraInfo._TYPE);
        cameraInfo.setDistortionModel("plumb_bob");
        cameraInfo.setD( new double[]{
                    // see /mnt/nixbig/data/project_AA1_2_extrinsics__phone_data_recording/VOS_data_2018_01_21_16_51_11/test_camera_pose_estimation_opencv_solvepnp.py
                    // see /mnt/nixbig/data/project_AA1_2_extrinsics__phone_data_recording/VOS_data_2018_01_21_16_51_11/intrinsics_matlab_3radial_2tangental_0skew.txt
                 0.004180016841640d, 0.136452931271259d ,           // k_1, k_2
                -0.001666231998527d, -0.00008160213039217031d ,     // p_1, p_2
                -0.638647134308425d                                 // k_3
        } );
        cameraInfo.setK( new double[]{                                            // calibrated at 352x288 per image passed to BoofCV by OpenCV JavaCameraFrame
                322.9596901156589d , 000.0000000000000d , 176.8267919600727d ,    //  f_x ,   0 , c_x
                000.0000000000000d , 323.8523693059909d , 146.7681514313797d ,    //    0 , f_y , c_y
                  0.0d ,               0.0d ,               1.0d                  //    0 ,   0 ,   1
        } );
        request.setCameraInfo(cameraInfo);
//        List<Observation2D3D> obs2D3D = new ArrayList<Observation2D3D>();
//        Observation2D3D obs = new william.chamberlain.androidvosopencvros.device.Observation2D3D();

//        int u_=100, v_=200;
//        double x_=11, y_=22, z_=33;
//        createObs(request, u_, v_, x_, y_, z_);
        createObs(request,     56.54614322277916d , 109.06295157200866d, 3.92d ,  1.60d , 0.645d  );  // 1 - 357
        createObs(request,     52.62135271557739d ,  77.59861887431173d, 3.92d ,  1.60d , 0.252d  );  // 1 - 257
        createObs(request,     182.8574242089062d , 108.62255574234761d, 3.92d ,  0.00d , 0.645d  );  // 2 - 357
        createObs(request,     182.41568510974201d , 76.8101381190807d,  3.92d ,  0.00d , 0.252d  );  // 2 - 257
        createObs(request,     285.879892804077d ,  108.32705433683117d, 3.92d , -1.28d , 0.645d  );  // 3 - 357
        createObs(request,     288.944522535763d ,   75.79036791913009d, 3.92d , -1.28d , 0.252d  );  // 3 - 257
        createObs(request,     297.48634983835746d , 92.40217046164975d, 2.64d , -0.96d , 0.645d  );  // 4 - 357
        createObs(request,     292.9941784546977d , 138.23436515146292d, 2.64d , -0.96d , 0.252d  );  // 4 - 257
        createObs(request,     312.2281500120752d , 214.16417888299193d, 1.36d , -0.64d , 0.645d  );  // 5 - 357
        createObs(request,     312.2390321889994d , 214.18113331292741d, 1.36d , -0.64d , 0.645d  );  // 5 - 357
        createObs(request,     246.04851963779973d ,214.0295061185083d,  1.36d , -0.32d , 0.645d  );  // 6 - 357
        createObs(request,     251.05223417229467d ,136.82524151860272d, 1.36d , -0.32d , 0.252d  );  // 6 - 257
        createObs(request,     180.1327822481631d , 213.4655434325827d,  1.36d ,  0.00d , 0.645d  );  // 7 - 357
        createObs(request,     179.24505599894138d ,136.39673251911438d, 1.36d ,  0.00d , 0.252d  );  // 7 - 257
        createObs(request,     50.91477856287188d , 212.58922727425147d, 1.36d ,  0.64d , 0.645d  );  // 8 - 357
        createObs(request,     39.748161293765655d ,136.19683774723288d, 1.36d ,  0.64d , 0.252d  );  // 8 - 257
        createObs(request,     37.093278687336245d ,138.83215031041536d, 2.64d ,  1.28d , 0.645d  );  // 9 - 357
        createObs(request,     29.93565957697492d ,  94.52635225774982d, 2.64d ,  1.28d , 0.252d  );  // 9 - 257
        createObs(request,     182.30592540117604d, 138.4954591343754d,  2.64d ,  0.00d , 0.645d  );  //10 - 357
        createObs(request,     181.84473615057965d , 93.39524995177273d, 2.64d ,  0.00d , 0.252d  );  //10 - 257
        poseFrom2D3D_serviceClient.call(request, poseFrom2D3D_responseListener);

        System.out.println("VosTaskAssignmentSubscriberNode: atestMatlabPoseEstimation_kitchen_realPixels: end");
    }

    /**
     * Create a feature observation with one 2D point and one 3D point, and add it to the service request.
     * @param request service request to add the observation to - updated.
     * @param u_ observed pixel position lateral:  (u,v) = (0,0) at the top left of the image, and (u,v) = (width,height) at the bottom right.
     * @param v_ observed pixel position vertical: (u,v) = (0,0) at the top left of the image, and (u,v) = (width,height) at the bottom right.
     * @param x_ observed 3D forward, in the robot/map/world coordinate frame.
     * @param y_ observed 3D left, in the robot/map/world coordinate frame.
     * @param z_ observed 3D up, in the robot/map/world coordinate frame.
     */
    private void createObs(PoseFrom2D3DRequest request, final double u_, final double v_,   final double x_, final double y_, final double z_) {
        geometry_msgs.Point point3D = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
        point3D.setX(x_);  point3D.setY(y_);  point3D.setZ(z_);
        Observation2D3D obs = connectedNode.getTopicMessageFactory().newFromType(Observation2D3D._TYPE);
        obs.setU(u_);  obs.setV(v_);
        obs.setPoint3D(point3D);
        obs.setImageTime(DateAndTime.nowAsTime());
        request.getPoints().add(obs);
    }

    //--- configuration ----------------------------------

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

    /*** initialise node: set up all the service clients, publishers, and subscribers **********************************/

    public static final String POSE_FROM_2D3D_SERVICE_ROS_NAME = "observations_to_pose_estimates";
    private static boolean connected_to_POSE_FROM_2D3D_SERVICE_ROS = false;

    @Override
    public void onStart(ConnectedNode connectedNode_) {
        final Log log = connectedNode_.getLog();
        this.connectedNode = connectedNode_;

        connect_to_POSE_FROM_2D3D_SERVICE_ROS();

        startTransformListener();  // start listening to TF :  TODO : move this to a central/singleton TF node :- only want to spend CPU on one at a time

        subscriber = connectedNode_.newSubscriber(nodeNamespace+"/vos_task_assignment_subscriber", WhereIsAsPub._TYPE);
        subscriber.addMessageListener(new MessageListener<WhereIsAsPub>() {                                 // declare inline
            @Override
            public void onNewMessage(WhereIsAsPub message) {
                log.info("VosTaskAssignmentSubscriberNode: message received: \"" + message.getRobotId() + "\" | \"" + message.getRequestId() + "\" | \""+message.getAlgorithm()+"\" | \""+message.getDescriptor()+"\" | \"" + message.getReturnUrl() + "\"");
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
                    java.util.Date statusTime = DateAndTime.toJavaDate(rosTime);
                    List<GoalStatus> statusList = new ArrayList<GoalStatus>();
                    for (GoalStatus move_base_goalStatus : goalStatusArray.getStatusList()) {
                        if(move_base_goalStatus.getGoalId().getId().contains("/move_base")) {                         // TODO: hardcoding
                            statusList.add(move_base_goalStatus);
                            for (RobotStatusChangeListener changeListener : robotStatusChangeListeners) {
                                changeListener.robotStatusChange(statusTime,statusList);
                            }
                        }
                    }
                }
            }
        } );

        String robot_graph_prefix = ROBOT_GRAPHNAME_PREFIX;
        addSubscriptionToRobotPose(connectedNode_, robot_graph_prefix);

        /** GetTf service client *********/
        try {
            transformClient = connectedNode_.newServiceClient(GET_TF_SERVICE_NAME,GetTf._TYPE);
                System.out.println("VosTaskAssSubNod: onStart: transformClient: connectedNode_.newServiceClient: SUCCESS");
        } catch (ServiceNotFoundException e) {
                    connectedNode.getLog().error("VosTaskAssSubNod: onStart: transformClient: connectedNode_.newServiceClient: ERROR: No service "+ GET_TF_SERVICE_NAME +" of type "+ GetTf._TYPE);
                    System.out.println("VosTaskAssSubNod: onStart: transformClient: connectedNode_.newServiceClient: ERROR: "+e); e.printStackTrace();
                    connectedNode.getLog().error("VosTaskAssSubNod: onStart: transformClient: connectedNode_.newServiceClient: ERROR: "+e, e);
            throw new RosRuntimeException(e);
        } catch (Exception e) {
            if (connectedNode != null) {
                        System.out.println("VosTaskAssSubNod: onStart: transformClient: connectedNode_.newServiceClient: ERROR: "+e); e.printStackTrace();
                        connectedNode.getLog().error("VosTaskAssSubNod: onStart: transformClient: connectedNode_.newServiceClient: ERROR: "+e, e);
            } else {
                        System.out.println("VosTaskAssSubNod: onStart: transformClient: connectedNode_.newServiceClient: ERROR: connectedNode_ == null: "+e);
                        System.out.println("VosTaskAssSubNod: onStart: transformClient: connectedNode_.newServiceClient: ERROR: connectedNode_ == null: "+e); e.printStackTrace();
                        e.printStackTrace();
            }
        }
    }

    private void connect_to_POSE_FROM_2D3D_SERVICE_ROS() {
        if(null !=poseFrom2D3D_serviceClient) {
            return;
        }
        if(connected_to_POSE_FROM_2D3D_SERVICE_ROS) {
            return;
        }
        try {
            poseFrom2D3D_serviceClient = connectedNode.newServiceClient(POSE_FROM_2D3D_SERVICE_ROS_NAME, PoseFrom2D3D._TYPE);
            poseFrom2D3D_responseListener = new PoseFrom2D3DResponseListener();
            connected_to_POSE_FROM_2D3D_SERVICE_ROS = true;
            System.out.println("VosTaskAssignmentSubscriberNode: onStart: poseFrom2D3D: setup successfully.");
        } catch (ServiceNotFoundException e) {
            System.out.println("VosTaskAssignmentSubscriberNode: onStart: poseFrom2D3D: ServiceNotFoundException for '"+POSE_FROM_2D3D_SERVICE_ROS_NAME +"': "+e.getMessage());
            e.printStackTrace();
            throw new RosRuntimeException(e);
        } catch (Exception e) {
            if (connectedNode != null) {
                System.out.println("VosTaskAssignmentSubscriberNode: onStart: poseFrom2D3D: Exception for '" + POSE_FROM_2D3D_SERVICE_ROS_NAME + "': " + e.getMessage());
                e.printStackTrace();
                throw new RosRuntimeException(e);
            } else {
                System.out.println("VosTaskAssignmentSubscriberNode: onStart: poseFrom2D3D: connectedNode_ == null: " + e.getMessage());
                e.printStackTrace();
            }
        }
    }

    public void getTfDummy(String sourceFrameId_, String targetFrameId_, java.util.Date time_) {
        if (null==transformClient) {
            System.out.println("WARN: getTfDummy: null==transformClient");
            return;
        }
        GetTfRequest getTfRequest = transformClient.newMessage();

        std_msgs.String sourceFrameId = getTfRequest.getSourceFrameId();
        sourceFrameId.setData(sourceFrameId_);
        getTfRequest.setSourceFrameId(sourceFrameId);

        std_msgs.String targetFrameId = getTfRequest.getTargetFrameId();
        targetFrameId.setData(targetFrameId_);
        getTfRequest.setTargetFrameId(targetFrameId);

        std_msgs.Time time = getTfRequest.getTime();
        time.setData(DateAndTime.toRosTime(time_));
        getTfRequest.setTime(time);
        GetTfListenerDummy getTfListener = new GetTfListenerDummy(time_);
        transformClient.call(getTfRequest, getTfListener);
    }

    /*** TF by request *******************************/
    public void getTf(String sourceFrameId_, String targetFrameId_, java.util.Date time_, PixelPosition pixelPosition_, Se3_F64 transformOfFeatureInVisualModel_, RobotPoseListener robotPoseListener_) {
        GetTfRequest getTfRequest = transformClient.newMessage();

        std_msgs.String sourceFrameId = getTfRequest.getSourceFrameId();
        sourceFrameId.setData(sourceFrameId_);
        getTfRequest.setSourceFrameId(sourceFrameId);

        std_msgs.String targetFrameId = getTfRequest.getTargetFrameId();
        targetFrameId.setData(targetFrameId_);
        getTfRequest.setTargetFrameId(targetFrameId);

        std_msgs.Time time = getTfRequest.getTime();
        time.setData(DateAndTime.toRosTime(time_));
        getTfRequest.setTime(time);

        //  GetTfListener(java.util.Date date_, PixelPosition pixelPosition_, Se3_F64 transformOfFeatureInVisualModel_, RobotPoseListener robotPoseListener_) {
        GetTfListener getTfListener = new GetTfListener(time_, pixelPosition_, transformOfFeatureInVisualModel_, robotPoseListener_);
        transformClient.call(getTfRequest, getTfListener);
    }

    /***************************************************************************/
    class Local_FrameTransform {
        private FrameTransform frameTransform;
        private boolean isSet = false;
        void setFrameTransform(FrameTransform frameTransform_) {
            this.frameTransform = frameTransform_;
            this.isSet = true;
        }
        boolean isSet() {return isSet;}
        FrameTransform frameTransform(){return frameTransform;}
    }

    /***************************************************************************/
    class GetTfListenerDummy implements ServiceResponseListener<GetTfResponse> {
        java.util.Date date = null;
        GetTfListenerDummy(java.util.Date date_) {
            this.date = date_;
        }
        @Override
        public void onSuccess(GetTfResponse getTfResponse) {
            System.out.println("GetTfListenerDummy: onSuccess: got response with transform="+getTfResponse.getTransformFound().getTransform()+" for robot pose at "+date+" = "+date.getTime()+"ms");
        }
        @Override
        public void onFailure(RemoteException e) {
            System.out.println("GetTfListenerDummy: onFailure: for robot pose at "+date+" = "+date.getTime()+"ms: exception: "+e);
            e.printStackTrace();
            //  throw new RuntimeException("GetTfListenerDummy: onFailure: ERROR: "+e, e);
        }
    }


    /***************************************************************************/
    class GetTfListener implements ServiceResponseListener<GetTfResponse> {
        java.util.Date date = null;
        PixelPosition pixelPosition = null;
        Se3_F64 transformOfFeatureInVisualModel = null;
        RobotPoseListener robotPoseListener = null;
        geometry_msgs.TransformStamped transformStamped = null;
        org.ros.rosjava_geometry.FrameTransform frameTransform = null;

        GetTfListener(java.util.Date date_, PixelPosition pixelPosition_, Se3_F64 transformOfFeatureInVisualModel_, RobotPoseListener robotPoseListener_) {
            this.date = date_;
            this.pixelPosition = pixelPosition_;
            this.transformOfFeatureInVisualModel = transformOfFeatureInVisualModel_;
            this.robotPoseListener = robotPoseListener_;
        }

        @Override
        public void onSuccess(GetTfResponse getTfResponse) {
            System.out.println("GetTfListener: onSuccess: got response with transform="+getTfResponse.getTransformFound().getTransform()+" for robot pose at "+date+" = "+date.getTime()+"ms");
//            this.local_FrameTransform.setFrameTransform(FrameTransform.fromTransformStampedMessage(getTfResponse.getTransformFound()));
            transformStamped = getTfResponse.getTransformFound();
            frameTransform   = FrameTransform.fromTransformStampedMessage(transformStamped);
            this.robotPoseListener.robotPoseObservation(this.date, this.pixelPosition, this.transformOfFeatureInVisualModel, frameTransform);
        }

        @Override
        public void onFailure(RemoteException e) {
            System.out.println("GetTfListener: onFailure with RemoteException "+e.getMessage());
            e.printStackTrace();
            // TODO - what to do?
            // throw new RuntimeException("GetTfListener: onFailure: ERROR: "+e, e);
        }
    }


    /*** TF/TF2 **************************************/
    /*** RosJava TF does not interpolate for time ****/
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
//                    poseListener.robotPose(robotId, pose);
                }
            }
        }
    }

    /*** RobotPoseMeasure *****************************************/

    public Transform askRobotForPose() {  // see SmartCameraExtrinsicsCalibrator

        //  https://answers.ros.org/question/250292/in-rosjavahow-can-i-transform-the-odom-frame-to-map-frame/
        //  https://answers.ros.org/question/249861/how-can-i-record-the-robot-trajectory-and-show-it-on-the-android-device/
        // obtain current robot pose
        FrameTransform frameTransform = frameTransformTree.transform(GraphName.of("/base_link"), GraphName.of("/map"));
        Transform robotPoseInMap = frameTransform.getTransform();

        return robotPoseInMap;

        // obtain current robot position in image
        // if not visible, set robot goal back to last position
        // if not have enough points to estimate floor homography, set another goal for the robot
        // estimate floor homography
        // if estimate is bad, set robot goal to gather more data
        // if estimate is good, done
    }
    public FrameTransform askRobotForPoseFrame() {  // see SmartCameraExtrinsicsCalibrator
        FrameTransform frameTransform = frameTransformTree.transform(GraphName.of("/base_link"), GraphName.of("/map"));
        return frameTransform;
    }

//    public FrameTransform askRobotForPoseFrameAsync(java.util.Date date) {  // see SmartCameraExtrinsicsCalibrator
//        FrameTransform local_FrameTransform = null;
//        local_FrameTransform = getTf("base_link", "map", date);
//        return local_FrameTransform;
//    }


    public void askRobotForPoseFrameAsync(String robotId_, java.util.Date imageFrameTime_, PixelPosition pixelPosition_, Se3_F64 transformOfFeatureInVisualModel_, RobotPoseListener robotPoseListener) {
        getTf("/map", "/base_link", imageFrameTime_,pixelPosition_,transformOfFeatureInVisualModel_,robotPoseListener);
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
        System.out.println("VosTaskAssignmentSubscriberNode: sendRobotGoalInWorldFrame(Transform transform_): start");
        PoseStamped poseStamped = robot_goal_publisher.newMessage();
        header(poseStamped, "map");
        RosTypes.updatePoseStampedFromTransform(transform_, poseStamped);

        robot_goal_publisher.publish(poseStamped);
        System.out.println("VosTaskAssignmentSubscriberNode: sendRobotGoalInWorldFrame(Transform transform_): end");
    }


    @Override // RobotGoalPublisher
    public void sendRobotGoalInRobotFrame(Transform transform_) {
        System.out.println("VosTaskAssignmentSubscriberNode: sendRobotGoalInRobotFrame(Transform transform_): start");
        PoseStamped poseStamped = robot_goal_publisher.newMessage();
        header(poseStamped, "base_link");
        RosTypes.updatePoseStampedFromTransform(transform_, poseStamped);

        robot_goal_publisher.publish(poseStamped);
        System.out.println("VosTaskAssignmentSubscriberNode: sendRobotGoalInRobotFrame(Transform transform_): end");
    }


}


