package william.chamberlain.androidvosopencvros;

import android.support.annotation.NonNull;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.service.ServiceServer;

import vos_aa1.LocaliseFromAFeature;
import vos_aa1.there_is_alg_desc;
import vos_aa1.there_is_alg_descRequest;
import vos_aa1.there_is_alg_descResponse;
import vos_aa1.where_is_alg_desc;
import vos_aa1.where_is_alg_descRequest;
import vos_aa1.where_is_alg_descResponse;

/**
 * Created by will on 11/07/17.
 */

public class WhereIs extends AbstractNodeMain {
    private ServiceServer<where_is_alg_descRequest, where_is_alg_descResponse> server;
    private ServiceClient<there_is_alg_descRequest, there_is_alg_descResponse> clientToRobot;
    private ThereIsResponseListener responseListener;
    private ConnectedNode connectedNode;
    private String nodeNamespace = "vision_os/";

    public void setNodeNamespace(java.lang.String nodeNamespace) {
        this.nodeNamespace = nodeNamespace;
        if(nodeNamespace.length()>0 && !nodeNamespace.endsWith("/")) { // if not empty, has to end with '/' ; implementation detail
            this.nodeNamespace = this.nodeNamespace+"/";
        }
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(nodeNamespace+"where_is__there_is");
    }
    @Override
    public void onStart(ConnectedNode connectedNode_) {
        connectedNode = connectedNode_;

        initialiseServer(serverNodeName(), where_is_alg_desc._TYPE);

        // initialiseClient(clientNodeName(), there_is_alg_desc._TYPE);  // TODO - only connect to the robot when making responses
    }

    @NonNull
    private String clientNodeName() {
        return nodeNamespace+"there_is";
    }

    @NonNull
    private String serverNodeName() {
        return nodeNamespace+"where_is";
    }

    private void initialiseServer(String nodeName_, String type_) {
        server = connectedNode.newServiceServer(
                nodeName_ ,
                LocaliseFromAFeature._TYPE,
                new ServiceResponseBuilder<where_is_alg_descRequest,where_is_alg_descResponse>() {
                    @Override
                    public void build(where_is_alg_descRequest request, where_is_alg_descResponse response)
                            throws ServiceException {
                        response.setAcknowledgment("OK");
                    }
                } );
    }

    private void initialiseClient(String nodeName_, String type_) {
        System.out.println(type_+": onStart");
        try {
            clientToRobot = connectedNode.newServiceClient(nodeName_, type_);  // TODO: un-hardcode the service URL
            responseListener = new ThereIsResponseListener(connectedNode);
        } catch (ServiceNotFoundException e) {
            e.printStackTrace();
            connectedNode.getLog().error(type_+": onStart: fail ServiceNotFoundException");
            throw new RosRuntimeException(e);
        } catch (Exception e) {
            if (connectedNode != null) {
                e.printStackTrace();
                connectedNode.getLog().fatal(e);
            } else {
                System.out.println(type_+": onStart: fail Exception");
                e.printStackTrace();
            }
        }
        connectedNode.getLog().info(type_+": initialiseClient: success");
    }


    class ThereIsResponseListener implements ServiceResponseListener<there_is_alg_descResponse> {
        ConnectedNode connectedNode;

        ThereIsResponseListener(ConnectedNode connectedNode_) {
            connectedNode = connectedNode_;
        }

        @Override
        public void onSuccess(there_is_alg_descResponse response) {
            connectedNode.getLog().info("ThereIsResponseListener: onSuccess");
        }

        @Override
        public void onFailure(RemoteException e) {
            connectedNode.getLog().error("ThereIsResponseListener: onFailure");
            e.printStackTrace();
            throw new RosRuntimeException(e);
        }
    }
}
