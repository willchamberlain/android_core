package william.chamberlain.androidvosopencvros;

import android.util.Log;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import java.util.regex.Matcher;

import geometry_msgs.Point;
import geometry_msgs.Quaternion;
import vos_aa1.LocaliseFromAFeature;
import vos_aa1.LocaliseFromAFeatureRequest;
import vos_aa1.LocaliseFromAFeatureResponse;
import vos_aa1.RegisterVisionSource;
import vos_aa1.RegisterVisionSourceRequest;
import vos_aa1.RegisterVisionSourceResponse;
import vos_aa1.VisualFeatureObservation;

import static william.chamberlain.androidvosopencvros.DataExchange.posePattern;

/**
 * Created by will on 7/03/17.
 */

public class RegisterVisionSourceClient extends AbstractNodeMain {
    private ServiceClient<RegisterVisionSourceRequest, RegisterVisionSourceResponse> serviceClient;
    private RegisterVisionSourceResponseListener responseListener;
    private ConnectedNode connectedNode;
    private PosedEntity posedEntity = null;
    private String baseUrl;
    private static final String TAG = "RegisterVisionSourceCli";

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("androidvosopencvros/register_vision_source");
    }

    @Override
    public void onStart(ConnectedNode connectedNode_) {
        System.out.println("RegisterVisionSourceClient: onStart");
        connectedNode = connectedNode_;
        connectServiceClient();
        System.out.println("RegisterVisionSourceClient: onStart: success");
        Log.i(TAG, "onStart: success");
    }

    private void connectServiceClient() {
        try {
            serviceClient = connectedNode.newServiceClient("/androidvosopencvros/register_vision_source", RegisterVisionSource._TYPE);  // TODO: un-hardcode the service URL
            responseListener = new RegisterVisionSourceResponseListener(connectedNode, posedEntity);
        } catch (ServiceNotFoundException e) {
            System.out.println("RegisterVisionSourceClient: onStart: fail ServiceNotFoundException");
            e.printStackTrace();
            connectedNode.getLog().error("RegisterVisionSourceClient: onStart: fail ServiceNotFoundException");
            Log.e(TAG, "onStart: ServiceNotFoundException", e);
            throw new RosRuntimeException(e);
        } catch (Exception e) {
            if (connectedNode != null) {
                System.out.println("RegisterVisionSourceClient: onStart: fail Exception");
                e.printStackTrace();
                Log.e(TAG, "onStart: Exception: connectedNode_ != null: ", e);
                connectedNode.getLog().fatal(e);
            } else {
                System.out.println("RegisterVisionSourceClient: onStart: fail Exception");
                Log.e(TAG, "onStart: Exception: connectedNode_ == null: ", e);
                e.printStackTrace();
            }
        }
    }

    public void setBaseUrl(String baseUrl_) {
        this.baseUrl = baseUrl_;
    }

    public void registerVisionSource() {
        Log.i(TAG,"registerVisionSource: registering as a vision source.");
        if (null == serviceClient ) {
            Log.e(TAG,"registerVisionSource: CANNOT register as a vision source: serviceClient IS NULL.");
            return;
        }
        RegisterVisionSourceRequest serviceRequest = serviceClient.newMessage();
        serviceRequest.setVisionSourceBaseUrl(baseUrl);
        serviceClient.call(serviceRequest, responseListener);
    }

//    TODO: register as a vision source with a (set of) FoV
//    public void registerVisionSource() {
//        RegisterVisionSourceRequest serviceRequest = serviceClient.newMessage();
//        serviceRequest.setVisionSourceBaseUrl(baseUrl);
//        serviceClient.call(serviceRequest, responseListener);
//    }


    class RegisterVisionSourceResponseListener implements ServiceResponseListener<RegisterVisionSourceResponse> {
        ConnectedNode connectedNode;
        PosedEntity posedEntity = null;

        public RegisterVisionSourceResponseListener(ConnectedNode connectedNode_, PosedEntity posedEntity_) {
            connectedNode = connectedNode_;
            posedEntity = posedEntity_;
        }

        public void shutDown() {
            // placeholder for now.
            // may need to relinquish the connectedNode
        }

        /**
         * Updates the vision source/PosedEntity pose if the response contains a vision source pose
         * - currently defined as a string in DataExchange.posePattern format.
         * @param response
         */
        @Override
        public void onSuccess(RegisterVisionSourceResponse response) {
            connectedNode.getLog().info("RegisterVisionSourceResponseListener: onSuccess");   //      String.format("%d + %d = %d", request.getA(), request.getB(), response.getSum()));    - can't tie the response to the request
            System.out.println("RegisterVisionSourceResponseListener: onSuccess");
            if(response.getAcknowledgement().contains("pose")) {
                if(null==posedEntity) {
                    System.out.println("RegisterVisionSourceResponseListener: onSuccess: cannot set the PosedEntity pose, because the PosedEntity is null;");
                    return;
                }
                /*
                */
                Matcher matcher = posePattern.matcher(response.getAcknowledgement());
                if(matcher.matches()) {
                    double x = Double.parseDouble(matcher.group(2));
                    double y = Double.parseDouble(matcher.group(3));
                    double z = Double.parseDouble(matcher.group(4));
                    double qx = Double.parseDouble(matcher.group(5));
                    double qy = Double.parseDouble(matcher.group(6));
                    double qz = Double.parseDouble(matcher.group(7));
                    double qw = Double.parseDouble(matcher.group(8));
                    posedEntity.setPose(new double[]{x, y, z}, new double[]{qx, qy, qz, qw});
                }
            }
        }

        @Override
        public void onFailure(RemoteException e) {
            connectedNode.getLog().error("RegisterVisionSourceResponseListener: onFailure");   // - can't tie the response to the request - see http://rosjava.github.io/rosjava_core/0.1.6/getting_started.html for an alternative
            System.out.println("RegisterVisionSourceResponseListener: onFailure");
            e.printStackTrace();
            throw new RosRuntimeException(e);
        }

    }

    public void setPosedEntity(PosedEntity posedEntity) {
        this.posedEntity = posedEntity;
    }

}