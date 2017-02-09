package william.chamberlain.androidvosopencvros;


import android.os.Looper;
import android.os.SystemClock;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import sensor_msgs.FluidPressure;
import sensor_msgs.MultiDOFJointState;

/**
 * Created by will on 8/02/17.
 */

public class AprilTagsPosePublisher implements NodeMain {
    private Publisher<std_msgs.String> publisher;

    @Override
    public GraphName getDefaultNodeName() {
            return GraphName.of("androidvosopencvros/april_tags_publisher");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        System.out.println("AprilTagsPosePublisher: onStart");
        try {
            this.publisher = connectedNode.newPublisher("androidvosopencvros_samsung/april_tags_publisher", std_msgs.String._TYPE);
            System.out.println("AprilTagsPosePublisher: onStart: connected");

            connectedNode.executeCancellableLoop(new CancellableLoop() {
                private int sequenceNumber;

                @Override
                protected void setup() {
                    sequenceNumber = 0;
                }

                @Override
                protected void loop() throws InterruptedException {
                    if(tagid>=0) {
                        publishAprilTagId();
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

    public void publishAprilTagId(int apriltagid) {
        this.tagid = apriltagid;
    }

    public void publishAprilTagId() {
        System.out.println("AprilTagsPosePublisher: publishAprilTagId: start");
        std_msgs.String message_string = publisher.newMessage();
        message_string.setData(Integer.toString(tagid));
        publisher.publish(message_string);
        System.out.println("AprilTagsPosePublisher: publishAprilTagId: end");
    }

    @Override
    public void onShutdown(Node node) {
        System.out.println("AprilTagsPosePublisher: onShutdown");
    }

    @Override
    public void onShutdownComplete(Node node) {
        System.out.println("AprilTagsPosePublisher: onShutdownComplete");

    }

    @Override
    public void onError(Node node, Throwable throwable) {
        System.out.println("AprilTagsPosePublisher: onError");
    }
}
