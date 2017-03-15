package william.chamberlain.androidvosopencvros;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import geometry_msgs.Point;
import geometry_msgs.Quaternion;
import vos_aa1.LocaliseFromAFeature;
import vos_aa1.LocaliseFromAFeatureRequest;
import vos_aa1.LocaliseFromAFeatureResponse;
import vos_aa1.VisualFeatureObservation;

import static william.chamberlain.androidvosopencvros.Constants.APRIL_TAGS_KAESS_36_H_11;

/**
 * Created by will on 7/03/17.
 */

public class LocaliseFromAFeatureClient extends AbstractNodeMain {
    private ServiceClient<LocaliseFromAFeatureRequest, LocaliseFromAFeatureResponse> serviceClient;
    private LocaliseFromAFeatureResponseListener responseListener;
    private ConnectedNode connectedNode;
    private String cameraFrameId;
    private PosedEntity posedEntity;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("androidvosopencvros/localise_from_a_feature_client");
    }

    @Override
    public void onStart(ConnectedNode connectedNode_) {
        System.out.println("LocaliseFromAFeatureClient: onStart");
        try {
            serviceClient = connectedNode_.newServiceClient("/androidvosopencvros/localise_from_a_feature", LocaliseFromAFeature._TYPE);  // TODO: un-hardcode the service URL
            connectedNode = connectedNode_;
            responseListener = new LocaliseFromAFeatureResponseListener(connectedNode, posedEntity);
        } catch (ServiceNotFoundException e) {
            System.out.println("LocaliseFromAFeatureClient: onStart: fail ServiceNotFoundException");
            e.printStackTrace();
            connectedNode.getLog().error("LocaliseFromAFeatureClient: onStart: fail ServiceNotFoundException");
            throw new RosRuntimeException(e);
        } catch (Exception e) {
            if (connectedNode_ != null) {
                System.out.println("LocaliseFromAFeatureClient: onStart: fail Exception");
                e.printStackTrace();
                connectedNode_.getLog().fatal(e);
            } else {
                System.out.println("LocaliseFromAFeatureClient: onStart: fail Exception");
                e.printStackTrace();
            }
        }
        System.out.println("LocaliseFromAFeatureClient: onStart: success");
    }

    public void localiseFromAFeature(int tagId, double x, double y, double z, double qx, double qy, double qz, double qw) {
        LocaliseFromAFeatureRequest serviceRequest = serviceClient.newMessage();
        VisualFeatureObservation visualFeature = serviceRequest.getVisualFeature();

        Quaternion featureOrientation = visualFeature.getPose().getPose().getOrientation(); //  featureOrientationRelativeToCameraCentreFrame;
        Geometry.applyQuaternionParams(qx, qy, qz, qw, featureOrientation);

        Point translationToFeature = visualFeature.getPose().getPose().getPosition();
        Geometry.applyTranslationParams(x, y, z, translationToFeature);

        visualFeature.setAlgorithm(APRIL_TAGS_KAESS_36_H_11);
        visualFeature.setId(tagId);

        visualFeature.getPose().getHeader().setFrameId(cameraFrameId);

        serviceClient.call(serviceRequest, responseListener);

    }


    class LocaliseFromAFeatureResponseListener implements ServiceResponseListener<LocaliseFromAFeatureResponse> {
        ConnectedNode connectedNode;
        PosedEntity posedEntity;

        public LocaliseFromAFeatureResponseListener(ConnectedNode connectedNode_, PosedEntity posedEntity_) {
            connectedNode = connectedNode_;
            posedEntity = posedEntity_;
        }

        public void shutDown() {
            // placeholder for now.
            // may need to relinquish the connectedNode
        }

        @Override
        public void onSuccess(LocaliseFromAFeatureResponse response) {
            connectedNode.getLog().info("LocaliseFromAFeatureResponseListener: onSuccess");   //      String.format("%d + %d = %d", request.getA(), request.getB(), response.getSum()));    - can't tie the response to the request
            System.out.println("LocaliseFromAFeatureResponseListener: onSuccess");
//  TODO - this closes the loop to set the camera pose as the location supplied by the service
            if (response.getPoseFound()) {
                connectedNode.getLog().info("LocaliseFromAFeatureResponseListener: pose found");
                connectedNode.getLog().info("LocaliseFromAFeatureResponseListener: pose found = ");
                connectedNode.getLog().info(response.getPose());
                posedEntity.setPose(response.getPose());
            } else {
                connectedNode.getLog().info("LocaliseFromAFeatureResponseListener: NO POSE FOUND");
            }
        }

        @Override
        public void onFailure(RemoteException e) {
            connectedNode.getLog().error("LocaliseFromAFeatureResponseListener: onFailure");   // - can't tie the response to the request - see http://rosjava.github.io/rosjava_core/0.1.6/getting_started.html for an alternative
            System.out.println("LocaliseFromAFeatureResponseListener: onFailure");
            e.printStackTrace();
            throw new RosRuntimeException(e);
        }

    }

    public void setCameraFrameId(String cameraFrameId) {
        this.cameraFrameId = cameraFrameId;
    }

    public void setPosedEntity(PosedEntity posedEntity) {
        this.posedEntity = posedEntity;
    }
}