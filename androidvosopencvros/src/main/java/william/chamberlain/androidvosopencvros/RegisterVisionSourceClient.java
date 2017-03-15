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

import geometry_msgs.Point;
import geometry_msgs.Quaternion;
import vos_aa1.LocaliseFromAFeature;
import vos_aa1.LocaliseFromAFeatureRequest;
import vos_aa1.LocaliseFromAFeatureResponse;
import vos_aa1.RegisterVisionSource;
import vos_aa1.RegisterVisionSourceRequest;
import vos_aa1.RegisterVisionSourceResponse;
import vos_aa1.VisualFeatureObservation;

/**
 * Created by will on 7/03/17.
 */

public class RegisterVisionSourceClient extends AbstractNodeMain {
    private ServiceClient<RegisterVisionSourceRequest, RegisterVisionSourceResponse> serviceClient;
    private RegisterVisionSourceResponseListener responseListener;
    private ConnectedNode connectedNode;
    private String baseUrl;
    private static final String TAG = "RegisterVisionSourceCli";

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("androidvosopencvros/register_vision_source");
    }

    @Override
    public void onStart(ConnectedNode connectedNode_) {
        System.out.println("RegisterVisionSourceClient: onStart");
        try {
            serviceClient = connectedNode_.newServiceClient("/androidvosopencvros/register_vision_source", RegisterVisionSource._TYPE);  // TODO: un-hardcode the service URL
            connectedNode = connectedNode_;
            responseListener = new RegisterVisionSourceResponseListener(connectedNode);
        } catch (ServiceNotFoundException e) {
            System.out.println("RegisterVisionSourceClient: onStart: fail ServiceNotFoundException");
            e.printStackTrace();
            connectedNode.getLog().error("RegisterVisionSourceClient: onStart: fail ServiceNotFoundException");
            throw new RosRuntimeException(e);
        } catch (Exception e) {
            if (connectedNode_ != null) {
                System.out.println("RegisterVisionSourceClient: onStart: fail Exception");
                e.printStackTrace();
                connectedNode_.getLog().fatal(e);
            } else {
                System.out.println("RegisterVisionSourceClient: onStart: fail Exception");
                e.printStackTrace();
            }
        }
        System.out.println("RegisterVisionSourceClient: onStart: success");
    }

    public void setBaseUrl(String baseUrl_) {
        this.baseUrl = baseUrl_;
    }

    public void registerVisionSource() {
        Log.i(TAG,"registerVisionSource: registering as a vision source.");
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

        public RegisterVisionSourceResponseListener(ConnectedNode connectedNode_) {
            connectedNode = connectedNode_;
        }

        public void shutDown() {
            // placeholder for now.
            // may need to relinquish the connectedNode
        }

        @Override
        public void onSuccess(RegisterVisionSourceResponse response) {
            connectedNode.getLog().info("RegisterVisionSourceResponseListener: onSuccess");   //      String.format("%d + %d = %d", request.getA(), request.getB(), response.getSum()));    - can't tie the response to the request
            System.out.println("RegisterVisionSourceResponseListener: onSuccess");
        }

        @Override
        public void onFailure(RemoteException e) {
            connectedNode.getLog().error("RegisterVisionSourceResponseListener: onFailure");   // - can't tie the response to the request - see http://rosjava.github.io/rosjava_core/0.1.6/getting_started.html for an alternative
            System.out.println("RegisterVisionSourceResponseListener: onFailure");
            e.printStackTrace();
            throw new RosRuntimeException(e);
        }

    }

}