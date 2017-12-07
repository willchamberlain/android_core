package william.chamberlain.androidvosopencvros;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.MessageFactory;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import geometry_msgs.Point;
import geometry_msgs.PoseStamped;
import geometry_msgs.Quaternion;
import geometry_msgs.Twist;
import vos_aa1.DetectedFeature;
import vos_aa1.DetectedFeatureRequest;
import vos_aa1.DetectedFeatureResponse;
import vos_aa1.DetectedFeaturesRequest;
import vos_aa1.DetectedFeaturesResponse;
import vos_aa1.VisualFeatureObservation;

/**
 * Created by will on 16/02/17.
 */

public class DetectedFeaturesClient extends AbstractNodeMain {
    public static final String DETECTED_FEATURE_ROS_SERVICE_NAME = "/androidvosopencvros/detected_feature";
    private ServiceClient<DetectedFeaturesRequest, DetectedFeaturesResponse> serviceClient;
    private ReportDetectedFeaturesResponseListener responseListener;
    private ServiceClient<DetectedFeatureRequest, DetectedFeatureResponse> featureServiceClient;
    private ReportDetectedFeatureResponseListener featureResponseListener;
    private ConnectedNode connectedNode;
    private String cameraFrameId;
    private PosedEntity posedEntity;

    public void setCameraFrameId(String cameraFrameId_) {
        this.cameraFrameId = cameraFrameId_;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("androidvosopencvros/detected_feature_client");
    }

    @Override
    public void onStart(ConnectedNode connectedNode_) {
        System.out.println("DetectedFeaturesClient: onStart");
        try {
//            serviceClient = connectedNode.newServiceClient("/androidvosopencvros/detected_features", DetectedFeatures._TYPE);
//            responseListener = new ReportDetectedFeaturesResponseListener();
//            responseListener.setConnectedNode(connectedNode);
            featureServiceClient = connectedNode_.newServiceClient(DETECTED_FEATURE_ROS_SERVICE_NAME, DetectedFeature._TYPE);  // TODO: un-hardcode the service URL
            featureResponseListener = new ReportDetectedFeatureResponseListener();
            featureResponseListener.setConnectedNode(connectedNode_);
            connectedNode = connectedNode_;
        } catch (ServiceNotFoundException e) {
            connectedNode.getLog().error("DetectedFeaturesClient: No service "+DETECTED_FEATURE_ROS_SERVICE_NAME+" of type "+DetectedFeature._TYPE);
            System.out.println("DetectedFeaturesClient: onStart: fail ServiceNotFoundException");
            e.printStackTrace();
            connectedNode.getLog().error("DetectedFeaturesClient: onStart: fail ServiceNotFoundException");
            throw new RosRuntimeException(e);
        } catch (Exception e) {
            if (connectedNode_ != null) {
                System.out.println("DetectedFeaturesClient: onStart: fail Exception");
                e.printStackTrace();
                connectedNode_.getLog().fatal(e);
            } else {
                System.out.println("DetectedFeaturesClient: onStart: fail Exception");
                e.printStackTrace();
            }
        }
//        connectedNode.getLog().info("DetectedFeaturesClient: onStart: success");
        System.out.println("DetectedFeaturesClient: onStart: success");
    }

    public DetectedFeatureRequest emptyRequest() {
        return featureServiceClient.newMessage();
    }


    /** Reports the detected feature pose in the camera coordinate frame/system, and the camera's current pose to the VOS Server.
     *
     * @param x translation from camera centre frame to the feature - per Kaess AprilTag library.
     * @param y translation from camera centre frame to the feature - per Kaess AprilTag library.
     * @param z translation from camera centre frame to the feature - per Kaess AprilTag library.
     * @param qx orientation of the feature relative to the camera centre frame, applied after the translation to the feature - per Kaess AprilTag library ; expressed as a quaternion because reasons.
     * @param qy orientation of the feature relative to the camera centre frame, applied after the translation to the feature - per Kaess AprilTag library ; expressed as a quaternion because reasons.
     * @param qz orientation of the feature relative to the camera centre frame, applied after the translation to the feature - per Kaess AprilTag library ; expressed as a quaternion because reasons.
     * @param qw orientation of the feature relative to the camera centre frame, applied after the translation to the feature - per Kaess AprilTag library ; expressed as a quaternion because reasons.
     */
    public void reportDetectedFeature(int tagId, double x,double y,double z,double qx,double qy,double qz,double qw) {
        DetectedFeatureRequest serviceRequest = featureServiceClient.newMessage();

        applyCurrentPoseAsCameraPoseForDetectedFeature(serviceRequest);                             // sets DetectedFeatureRequest.cameraPose from PosedEntity pose.

        applyDetectionPoseAndIdForDetectedFeature(tagId, x, y, z, qx, qy, qz, qw, serviceRequest);  // sets DetectedFeatureRequest.visualFeature pose of the marker.

        featureServiceClient.call(serviceRequest,featureResponseListener);
    }

    public void justPublishPose(int tagId, double x,double y,double z,double qx,double qy,double qz,double qw)  {
        DetectedFeatureRequest serviceRequest = featureServiceClient.newMessage();
        VisualFeatureObservation visualFeature = serviceRequest.getVisualFeature();

        visualFeature.setAlgorithm(Algorithm.APRIL_TAGS_KAESS_36_H_11.canonicalName());
        visualFeature.setId(tagId);

        Quaternion featureOrientation = visualFeature.getPose().getPose().getOrientation(); //  featureOrientationRelativeToCameraCentreFrame;
        Geometry.applyQuaternionParams(qx, qy, qz, qw, featureOrientation);
        Point translationToFeature = visualFeature.getPose().getPose().getPosition();
        Geometry.applyTranslationParams(x, y, z, translationToFeature);

        serviceRequest.setVisualFeature(visualFeature);
        featureServiceClient.call(serviceRequest,featureResponseListener);
    }

    /** Reports the detected feature pose in the world coordinate frame/system, applying the camera's current pose to the detected feature's pose, before reporting to the VOS Server.
     *
     * @param x translation from camera centre frame to the feature - per Kaess AprilTag library.
     * @param y translation from camera centre frame to the feature - per Kaess AprilTag library.
     * @param z translation from camera centre frame to the feature - per Kaess AprilTag library.
     * @param qx orientation of the feature relative to the camera centre frame, applied after the translation to the feature - per Kaess AprilTag library ; expressed as a quaternion because reasons.
     * @param qy orientation of the feature relative to the camera centre frame, applied after the translation to the feature - per Kaess AprilTag library ; expressed as a quaternion because reasons.
     * @param qz orientation of the feature relative to the camera centre frame, applied after the translation to the feature - per Kaess AprilTag library ; expressed as a quaternion because reasons.
     * @param qw orientation of the feature relative to the camera centre frame, applied after the translation to the feature - per Kaess AprilTag library ; expressed as a quaternion because reasons.
     */
    public void reportDetectedFeatureInWorldFrame(int tagId, double x,double y,double z,double qx,double qy,double qz,double qw) {
        System.out.println("reportDetectedFeatureInWorldFrame: tagId="+tagId+" cameraFrameId="+cameraFrameId);
        DetectedFeatureRequest serviceRequest = featureServiceClient.newMessage();

        applyCurrentPoseAsCameraPoseForDetectedFeature(serviceRequest);                             // sets DetectedFeatureRequest.cameraPose from PosedEntity pose.

        applyDetectionPoseAndIdForDetectedFeature(tagId, x, y, z, qx, qy, qz, qw, serviceRequest);  // sets DetectedFeatureRequest.visualFeature pose of the marker.

        featureServiceClient.call(serviceRequest,featureResponseListener);
    }

    /** Sets the marker id and pose of the detected feature in the DetectedFeatureRequest.
     *
     * @param tagId marker id.
     * @param x DetectedFeatureRequest.visualFeature.pose position x.
     * @param y DetectedFeatureRequest.visualFeature.pose position y.
     * @param z DetectedFeatureRequest.visualFeature.pose position z.
     * @param qx DetectedFeatureRequest.visualFeature.pose orientation quaternion x.
     * @param qy DetectedFeatureRequest.visualFeature.pose orientation quaternion y.
     * @param qz DetectedFeatureRequest.visualFeature.pose orientation quaternion z.
     * @param qw DetectedFeatureRequest.visualFeature.pose orientation quaternion w.
     * @param serviceRequest the DetectedFeatureRequest updated.
     */
    private void applyDetectionPoseAndIdForDetectedFeature(int tagId, double x, double y, double z, double qx, double qy, double qz, double qw, DetectedFeatureRequest serviceRequest) {
        VisualFeatureObservation visualFeature = serviceRequest.getVisualFeature();

        visualFeature.setAlgorithm(Algorithm.APRIL_TAGS_KAESS_36_H_11.canonicalName());
        visualFeature.setId(tagId);

        Quaternion featureOrientation = visualFeature.getPose().getPose().getOrientation(); //  featureOrientationRelativeToCameraCentreFrame;
        Geometry.applyQuaternionParams(qx, qy, qz, qw, featureOrientation);
        Point translationToFeature = visualFeature.getPose().getPose().getPosition();
        Geometry.applyTranslationParams(x, y, z, translationToFeature);

        serviceRequest.setVisualFeature(visualFeature);
    }

    /** Sets the DetectedFeatureRequest's camera pose to the pose of the  posedEntity.
     * @param serviceRequest the DetectedFeatureRequest to set the camera pose in : serviceRequest.setCameraPose is updated.
     */
    private void applyCurrentPoseAsCameraPoseForDetectedFeature(DetectedFeatureRequest serviceRequest) {  // TODO ?
        PoseStamped cameraPose = serviceRequest.getCameraPose();
        cameraPose.getHeader().setFrameId(cameraFrameId);
        Quaternion cameraOrientationInWorld = cameraPose.getPose().getOrientation();
        Geometry.applyQuaternionParams(posedEntity.getOrientationQuaternionXyzw(), cameraOrientationInWorld);  // TODO ?
        Point cameraPositionInWorld = cameraPose.getPose().getPosition();
        Geometry.applyTranslationParams(posedEntity.getPositionXyz(), cameraPositionInWorld);
        serviceRequest.setCameraPose(cameraPose);
    }


    public void reportDetectedFeatures(geometry_msgs.PoseStamped cameraPose_, java.util.List<vos_aa1.VisualFeatureObservation> visualFeaturesToReport_) {
        System.out.println("DetectedFeaturesClient: reportDetectedFeatures: start");
        DetectedFeaturesRequest serviceRequest = serviceClient.newMessage();
        PoseStamped cameraPoseInRequest = serviceRequest.getCameraPose();
        cameraPoseInRequest.getHeader().setFrameId("/map");
        serviceRequest.getVisualFeatures();
        serviceRequest.setCameraPose(cameraPoseInRequest);
        serviceRequest.setVisualFeatures(visualFeaturesToReport_);
        serviceClient.call(serviceRequest,responseListener);
        System.out.println("DetectedFeaturesClient: reportDetectedFeatures: end");

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();                                    // see http://answers.ros.org/question/41336/creating-composite-rosjava-messages-without-a-node/
        MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
        Twist twist = messageFactory.newFromType(Twist._TYPE);
        twist.getAngular().setZ(0.5);

        MessageFactory serviceRequestMessageFactory = nodeConfiguration.getServiceRequestMessageFactory();      // see http://answers.ros.org/question/41336/creating-composite-rosjava-messages-without-a-node/
        DetectedFeaturesRequest dfr = serviceRequestMessageFactory.newFromType(DetectedFeaturesRequest._TYPE);
    }

    @Override
    public void onShutdown(Node node) {
        connectedNode.getLog().info("DetectedFeaturesClient: onShutdown");
        if(null != responseListener) { responseListener.shutDown(); }
    }

    @Override
    public void onShutdownComplete(Node node) {
        connectedNode.getLog().info("DetectedFeaturesClient: onShutdownComplete");

    }

    @Override
    public void onError(Node node, Throwable throwable) {
        throwable.printStackTrace();
        connectedNode.getLog().error("DetectedFeaturesClient: onError");
    }


    class ReportDetectedFeaturesResponseListener implements ServiceResponseListener<DetectedFeaturesResponse> {
        ConnectedNode connectedNode;

        public void setConnectedNode(ConnectedNode connectedNode_) {
            connectedNode = connectedNode_;
        }

        public void shutDown(){
            // placeholder for now.
            // may need to relinquish the connectedNode
        }

        @Override
        public void onSuccess(DetectedFeaturesResponse response) {
            connectedNode.getLog().info("ReportDetectedFeaturesResponseListener: onSuccess");   //      String.format("%d + %d = %d", request.getA(), request.getB(), response.getSum()));    - can't tie the response to the request
            System.out.println("ReportDetectedFeaturesResponseListener: onSuccess");
        }

        @Override
        public void onFailure(RemoteException e) {
            connectedNode.getLog().error("ReportDetectedFeaturesResponseListener: onFailure");   // - can't tie the response to the request - see http://rosjava.github.io/rosjava_core/0.1.6/getting_started.html for an alternative
            System.out.println("ReportDetectedFeaturesResponseListener: onFailure");
            e.printStackTrace();
            throw new RosRuntimeException(e);
        }
    }


    /** No response expected, just an acknowledgement: takes no actions, simply logs the return text; does not make any recovery from service failure. */
    class ReportDetectedFeatureResponseListener implements ServiceResponseListener<DetectedFeatureResponse> {
        ConnectedNode connectedNode;

        public void setConnectedNode(ConnectedNode connectedNode_) {
            connectedNode = connectedNode_;
        }

        public void shutDown(){
            // placeholder for now.
            // may need to relinquish the connectedNode
        }

        @Override
        public void onSuccess(DetectedFeatureResponse response) {
            connectedNode.getLog().info("ReportDetectedFeatureResponseListener: onSuccess");   //      String.format("%d + %d = %d", request.getA(), request.getB(), response.getSum()));    - can't tie the response to the request
            System.out.println("ReportDetectedFeatureResponseListener: onSuccess");
        }

        @Override
        public void onFailure(RemoteException e) {
            connectedNode.getLog().error("ReportDetectedFeatureResponseListener: onFailure");   // - can't tie the response to the request - see http://rosjava.github.io/rosjava_core/0.1.6/getting_started.html for an alternative
            System.out.println("ReportDetectedFeatureResponseListener: onFailure");
            e.printStackTrace();
            throw new RosRuntimeException(e);
        }
    }

    public void setPosedEntity(PosedEntity posedEntity) {
        this.posedEntity = posedEntity;
    }

}
