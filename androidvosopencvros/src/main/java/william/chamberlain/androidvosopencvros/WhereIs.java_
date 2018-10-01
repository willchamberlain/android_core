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

import java.util.HashMap;

import geometry_msgs.PoseWithCovarianceStamped;
import vos_aa1.LocaliseFromAFeature;
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
    private HashMap<String, ServiceClient<there_is_alg_descRequest, there_is_alg_descResponse>> clientsToRobots
            = new HashMap<String, ServiceClient<there_is_alg_descRequest, there_is_alg_descResponse>>();
    private ServiceClient<there_is_alg_descRequest, there_is_alg_descResponse> clientToRobot;
    private ThereIsResponseListener responseFromRobotsListener;
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

        initialiseServerFromVosServer(serverNodeName(), where_is_alg_desc._TYPE);

        // initialiseClientToRobot(clientNodeName(), there_is_alg_desc._TYPE);  // TODO - only connect to the robot when making responses
    }

    @NonNull
    private String clientNodeName() {
        return nodeNamespace+"there_is";
    }

    @NonNull
    private String serverNodeName() {
        return nodeNamespace+"where_is";
    }

    private void initialiseServerFromVosServer(String nodeName_, String type_) {
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

    private void sendInformationToRobot(String robotUrl, PoseWithCovarianceStamped pose) {
        ServiceClient<there_is_alg_descRequest, there_is_alg_descResponse> client
                = getClientToRobot(robotUrl);
        there_is_alg_descRequest request = clientToRobot.newMessage();
        request.setPose(pose);
        client.call(request,responseFromRobotsListener);
    }

    private ServiceClient<there_is_alg_descRequest, there_is_alg_descResponse>
        getClientToRobot(String robotUrl) {
        ServiceClient<there_is_alg_descRequest, there_is_alg_descResponse> client
                = clientsToRobots.get(robotUrl);
        if(null == client) {
            client = addNewClientToRobotToSet(robotUrl);
        }
        return client;
    }

    private ServiceClient<there_is_alg_descRequest, there_is_alg_descResponse>
        addNewClientToRobotToSet(String robotUrl) {
        ServiceClient<there_is_alg_descRequest, there_is_alg_descResponse> client
                = initialiseClientToRobot(serverNodeName(), where_is_alg_desc._TYPE);
        clientsToRobots.put(robotUrl,client);
        return client;
    }

    private ServiceClient<there_is_alg_descRequest, there_is_alg_descResponse>
        initialiseClientToRobot(String nodeName_, String type_) {
        System.out.println(type_+": onStart");
        ServiceClient<there_is_alg_descRequest, there_is_alg_descResponse> newClientToRobot;
        try {
            newClientToRobot = connectedNode.newServiceClient(nodeName_, type_);  // TODO: un-hardcode the service URL
            responseFromRobotsListener = new ThereIsResponseListener(connectedNode);
            connectedNode.getLog().info(type_+": initialiseClientToRobot: success");
            return newClientToRobot;
        } catch (ServiceNotFoundException e) {
            e.printStackTrace();
            connectedNode.getLog().error(type_+": onStart: fail ServiceNotFoundException");
            throw new RosRuntimeException(e);
        } catch (Exception e) {
            if (connectedNode != null) {
                e.printStackTrace();
                connectedNode.getLog().fatal(e);
                throw new RosRuntimeException(e);
            } else {
                System.out.println(type_+": onStart: fail Exception");
                e.printStackTrace();
                throw new RosRuntimeException(e);
            }
        }
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
