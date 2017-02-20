package william.chamberlain.androidvosopencvros;

import org.ros.concurrent.CancellableLoop;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.internal.message.service.ServiceRequestMessageFactory;
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
import vos_aa1.DetectedFeatures;
import vos_aa1.DetectedFeaturesRequest;
import vos_aa1.DetectedFeaturesResponse;
import vos_aa1.VisualFeature;

/**
 * Created by will on 16/02/17.
 */

public class DetectedFeaturesClient extends AbstractNodeMain {
    private ServiceClient<DetectedFeaturesRequest, DetectedFeaturesResponse> serviceClient;
    private ReportDetectedFeaturesResponseListener responseListener;
    private ServiceClient<DetectedFeatureRequest, DetectedFeatureResponse> featureServiceClient;
    private ReportDetectedFeatureResponseListener featureResponseListener;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("androidvosopencvros/april_tags_publisher");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        System.out.println("DetectedFeaturesClient: onStart");
        try {
//            serviceClient = connectedNode.newServiceClient("/androidvosopencvros/detected_features", DetectedFeatures._TYPE);
//            responseListener = new ReportDetectedFeaturesResponseListener();
//            responseListener.setConnectedNode(connectedNode);
            featureServiceClient = connectedNode.newServiceClient("/androidvosopencvros/detected_feature", DetectedFeature._TYPE);
            featureResponseListener = new ReportDetectedFeatureResponseListener();
            featureResponseListener.setConnectedNode(connectedNode);
        } catch (ServiceNotFoundException e) {
            System.out.println("DetectedFeaturesClient: onStart: fail ServiceNotFoundException");
            throw new RosRuntimeException(e);
        } catch (Exception e) {
            if (connectedNode != null) {
                connectedNode.getLog().fatal(e);
            } else {
                System.out.println("DetectedFeaturesClient: onStart: fail Exception");
                e.printStackTrace();
            }
        }
        System.out.println("DetectedFeaturesClient: onStart: success");
    }

    /**
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

        PoseStamped cameraPose = serviceRequest.getCameraPose();
//        cameraPose.getHeader().setFrameId("/map");

        VisualFeature visualFeature = serviceRequest.getVisualFeature();
        Quaternion featureOrientation = visualFeature.getPose().getPose().getOrientation(); //  featureOrientationRelativeToCameraCentreFrame;
        featureOrientation.setX(qx);
        featureOrientation.setY(qy);
        featureOrientation.setZ(qz);
        featureOrientation.setW(qw);

        Point translationToFeature = visualFeature.getPose().getPose().getPosition();
        translationToFeature.setX(x);
        translationToFeature.setY(y);
        translationToFeature.setZ(z);

        visualFeature.setAlgorithm("AprilTags_Kaess_36h11");
        visualFeature.setId(tagId);

        serviceRequest.setCameraPose(cameraPose);
        serviceRequest.setVisualFeature(visualFeature);

        featureServiceClient.call(serviceRequest,featureResponseListener);
    }


    public void reportDetectedFeatures(geometry_msgs.PoseStamped cameraPose_, java.util.List<vos_aa1.VisualFeature> visualFeaturesToReport_) {
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
        System.out.println("DetectedFeaturesClient: onShutdown");
        responseListener.shutDown();
    }

    @Override
    public void onShutdownComplete(Node node) {
        System.out.println("DetectedFeaturesClient: onShutdownComplete");

    }

    @Override
    public void onError(Node node, Throwable throwable) {
        System.out.println("DetectedFeaturesClient: onError");
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
            connectedNode.getLog().info("ReportDetectedFeaturesResponseListener: onFailure");   // - can't tie the response to the request - see http://rosjava.github.io/rosjava_core/0.1.6/getting_started.html for an alternative
            System.out.println("ReportDetectedFeaturesResponseListener: onFailure");
            throw new RosRuntimeException(e);
        }
    }


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
            connectedNode.getLog().info("ReportDetectedFeatureResponseListener: onFailure");   // - can't tie the response to the request - see http://rosjava.github.io/rosjava_core/0.1.6/getting_started.html for an alternative
            System.out.println("ReportDetectedFeatureResponseListener: onFailure");
            throw new RosRuntimeException(e);
        }
    }


}
