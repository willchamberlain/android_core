package william.chamberlain.androidvosopencvros;

import org.ros.message.Duration;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import visualization_msgs.Marker;

import static william.chamberlain.androidvosopencvros.Constants.tagSize_metres;


/**
 * Created by will on 21/02/17.
 */
public class MarkerPublisherNode extends AbstractNodeMain {

    private Publisher<Marker> publisher;
    private String prefix;
//    public static final String MARKER_PUBLISHER_TOPIC_NAME

    public void setNodeNamespace(String NODE_NAMESPACE_) {
        prefix = NODE_NAMESPACE_;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("marker_publisher/marker_publisher");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        publisher = connectedNode.newPublisher(prefix+"/tag_marker_pub", Marker._TYPE);
    }

    public void publishMarker(String marker_namespace_, int marker_id_, String marker_text_, double x,double y,double z,double qx,double qy,double qz,double qw, String parent_frame_id, Time time_) {
        Marker marker = publisher.newMessage();

        marker.setNs(marker_namespace_);
        marker.setId(marker_id_);

        marker.getHeader().setStamp(time_);

        marker.setType(Marker.CUBE);

        marker.getScale().setX(0.010f);
        marker.getScale().setY(tagSize_metres);
        marker.getScale().setZ(tagSize_metres);
        marker.getColor().setR(0.0f);
        marker.getColor().setG(1.0f);
        marker.getColor().setB(0.0f);
        marker.getColor().setA(1.0f);

        marker.setAction(Marker.ADD);

        marker.getHeader().setFrameId(parent_frame_id);                      // use the frame_id of the parent, and the timestamp of the parent to allow synchronisation
        Geometry.applyTranslationParams(x, y, z, marker.getPose().getPosition());
        Geometry.applyQuaternionParams(qx, qy, qz, qw, marker.getPose().getOrientation());

        marker.setLifetime(Duration.MAX_VALUE);

        publisher.publish(marker);

        marker.setNs(marker_namespace_+"_label");
        marker.setType(marker.TEXT_VIEW_FACING);
        marker.setText(marker_text_);
        marker.setFrameLocked(false);
        marker.getScale().setZ(0.30f);

        publisher.publish(marker);
    }

}