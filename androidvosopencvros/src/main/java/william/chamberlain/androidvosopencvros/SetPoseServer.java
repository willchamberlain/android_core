package william.chamberlain.androidvosopencvros;

import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.topic.Publisher;

import geometry_msgs.TransformStamped;
import vos_aa1.SetPose;
import vos_aa1.SetPoseRequest;
import vos_aa1.SetPoseResponse;

/**
 * Runs on the smart camera/robot: accepts a pose for the smart camera/robot, which is propogated to the camera coordinate frame used for e.g. DetectedFeaturesClient through the robot model.
 *
 * Example command line call:
 * rosservice call /cam_1/set_pose "{control_point_name: bob, pose_in_map: {position: {x: 0.0, y: 0.0, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
 *
 * Created by will on 27/02/17.
 */

public class SetPoseServer extends AbstractNodeMain {
    private ConnectedNode connectedNode;
    private String cameraId;
    private String nodeNamespace;
    private double[] position = new double[3];
    private double[] orientation = new double[4];

    private PosedEntity posedEntity;

    public void setNodeNamespace(String NODE_NAMESPACE_) {
        nodeNamespace = NODE_NAMESPACE_;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("androidvosopencvros/set_pose_server");
    }

    public void cameraPoseFromControlPointPose(String controlPointName_) {
        // TODO - apply the robot model to the control_point_pose to get the camera pose, which is then the base pose for DetectedFeature ...
        // TODO - ... hold it; this should be taken care of by tf,
        //      if the robot/smart camera publishes the tf from
        //          e.g. cam_1/lens to cam_1/base_link to cam_1/camera,
        //          or e.g. r_1/left_wheel_contact to r_1/base_link to r_1/forward_camera  and  r_1/base_link to r_1/catadioptric_camera
        //      and DetectedFeature is in relation to e.g. cam_1/camera,  or  r_1/catadioptric_camera
    }

    protected void updateState() {
        posedEntity.setPose(position,orientation);
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
//        final Publisher<TransformStamped> broadcaster =
//                connectedNode.newPublisher(                     // see https://github.com/nickarmstrongcrews/rosjava-tf/blob/master/rosjava_tf_core/src/org/ros/rosjava/tf/pubsub/TransformBroadcaster.java
//                        "/tf",                                  // standard tf topic
//                        TransformStamped._TYPE);                // standard tf type
//        broadcaster.setLatchMode(true);

        connectedNode.newServiceServer(
                "/"+nodeNamespace+"/set_pose",  // TODO - remove hardcoding to base namespace '/'
                SetPose._TYPE,
                new ServiceResponseBuilder<SetPoseRequest, SetPoseResponse>() {

                    @Override
                    public void
                    build(SetPoseRequest request, SetPoseResponse response) {
                        System.out.println("SetPoseServer: ServiceResponseBuilder: build: received request for ControlPointName="+request.getControlPointName()+" position="+position+" orientation="+orientation);
                        request.getControlPointName();
                        position = new double[] {
                            request.getPoseInMap().getPosition().getX(),
                            request.getPoseInMap().getPosition().getY(),
                            request.getPoseInMap().getPosition().getZ()
                        };
                        orientation = new double[] {
                            request.getPoseInMap().getOrientation().getX(),
                            request.getPoseInMap().getOrientation().getY(),
                            request.getPoseInMap().getOrientation().getZ(),
                            request.getPoseInMap().getOrientation().getW()
                        };
                        updateState();

// tf to camera is published in detected_feature_server.py, when each feature tf is published
//                        TransformStamped txMsg = broadcaster.newMessage();
//                        //org.ros.message.geometry_msgs.TransformStamped txMsg = new org.ros.message.geometry_msgs.TransformStamped(); // see https://github.com/nickarmstrongcrews/rosjava-tf/blob/master/rosjava_tf_core/src/org/ros/rosjava/tf/pubsub/TransformBroadcaster.java
//                        txMsg.getHeader().setStamp( Date.nowAsTime() ); // TODO - what is this? - org.ros.message.Time.fromNano(t);
//                        txMsg.getHeader().setFrameId("map"); // parent
//                        txMsg.setChildFrameId(""); // child = this
//                        broadcaster.publish(txMsg);

                        response.setAcknowledgement(
                                "acknowledged: pose of the control point " + request.getControlPointName()
                                +" on " + nodeNamespace
                                +" is set as " + java.util.Arrays.toString(position)
                                +" and " + java.util.Arrays.toString(orientation));
                        System.out.println("SetPoseServer: ServiceResponseBuilder: build: sending response acknowledgement="+response.getAcknowledgement());
                    }
                });
    }


    public void setPosedEntity(PosedEntity posedEntity) {
        this.posedEntity = posedEntity;
    }
}
