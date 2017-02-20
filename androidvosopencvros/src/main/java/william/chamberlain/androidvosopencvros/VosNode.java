package william.chamberlain.androidvosopencvros;


import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.util.Random;

import std_msgs.Int32;
import std_msgs.String;

/**
 * Created by will on 8/02/17.
 */

public class VosNode implements NodeMain {
    private Publisher<std_msgs.String> robotLocationPublisher;
    private int robotTagId;

    @Override
    public GraphName getDefaultNodeName() {
            return GraphName.of("androidvosopencvros/vosnode");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        System.out.println("VosNode: onStart");
        try {
            Random randomNodeNamer = new Random();
            int nodeNameRandomPart = randomNodeNamer.nextInt(1000000);
            this.robotLocationPublisher = connectedNode.newPublisher("androidvosopencvros/vosnode_"+nodeNameRandomPart, std_msgs.String._TYPE);
            System.out.println("VosNode: onStart: connected robotLocationPublisher");

            Subscriber<std_msgs.Int32> subscriber = connectedNode.newSubscriber("androidvosopencvros/robotTagId", std_msgs.Int32._TYPE);
            subscriber.addMessageListener(new MessageListener<std_msgs.Int32>() {
                @Override
                public void onNewMessage(std_msgs.Int32 message) {
                    System.out.println("VosNode: robotTagListener: received new tag \"" + message.getData() + "\"");
                    robotTagId = message.getData();
                }
            });


            connectedNode.executeCancellableLoop(new CancellableLoop() {
                private int sequenceNumber;

                @Override
                protected void setup() {
                    sequenceNumber = 0;
                }

                @Override
                protected void loop() throws InterruptedException {
                    if(tagid>=0) {
                        publishRobotLocation();
                        tagid = -9000;
                    }
                    Thread.sleep(1000);
                }
            });
        } catch (Exception e) {
            if (connectedNode != null) {
                connectedNode.getLog().fatal(e);
            } else {
                e.printStackTrace();
            }
        }
    }

    int tagid = -9000;

    public void publishRobotLocation(int apriltagid) {
        this.tagid = apriltagid;
    }

    public void publishRobotLocation() {
        System.out.println("VosNode: publishRobotLocation: start");
        std_msgs.String message_string = robotLocationPublisher.newMessage();
        message_string.setData(Integer.toString(tagid));
        robotLocationPublisher.publish(message_string);
        System.out.println("VosNode: publishRobotLocation: end");
    }

    @Override
    public void onShutdown(Node node) {
        System.out.println("VosNode: onShutdown");
    }

    @Override
    public void onShutdownComplete(Node node) {
        System.out.println("VosNode: onShutdownComplete");

    }

    @Override
    public void onError(Node node, Throwable throwable) {
        System.out.println("VosNode: onError");
    }
}
