package william.chamberlain.androidvosopencvros.monitoring;


import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
//import cv_bridge.CvImage;
//import sensor_msgs.Image;

/**
 * Created by will on 8/02/17.
 */

public class ImagePublisher implements NodeMain {
    private Publisher<sensor_msgs.Image> publisher;

    @Override
    public GraphName getDefaultNodeName() {
            return GraphName.of("androidvosopencvros/image_publisher");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        System.out.println("ImagePublisher: onStart");
        try {
            this.publisher = connectedNode.newPublisher("androidvosopencvros/image_publisher", sensor_msgs.Image._TYPE);
            System.out.println("ImagePublisher: onStart: connected");

            connectedNode.executeCancellableLoop(new CancellableLoop() {
                private int sequenceNumber;

                @Override
                protected void setup() {
                    sequenceNumber = 0;
                }

                @Override
                protected void loop() throws InterruptedException {
                    if(tagid>=0) {
                        publishAprilTagId(tagid);
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

    public void publishImage() {
        System.out.println("ImagePublisher: publishRobotLocation: start");
        sensor_msgs.Image message_image = publisher.newMessage();
// TODO        message_image.setData();
        publisher.publish(message_image);
        System.out.println("ImagePublisher: publishRobotLocation: end");
    }

    @Override
    public void onShutdown(Node node) {
        System.out.println("ImagePublisher: onShutdown");
    }

    @Override
    public void onShutdownComplete(Node node) {
        System.out.println("ImagePublisher: onShutdownComplete");

    }

    @Override
    public void onError(Node node, Throwable throwable) {
        System.out.println("ImagePublisher: onError");
    }
}
