package william.chamberlain.androidvosopencvros.device;

import org.ros.internal.message.RawMessage;

import geometry_msgs.*;
import geometry_msgs.Point;
import geometry_msgs.Quaternion;

/**
 * Created by will on 31/08/17.
 */

public class Pose implements geometry_msgs.Pose {
    private william.chamberlain.androidvosopencvros.device.Point position;
    private william.chamberlain.androidvosopencvros.device.Quaternion orientation;

    public Pose(Point position_, Quaternion orientation_) {
        setPosition(position_);
        setOrientation(orientation_);
    }

    public Pose(geometry_msgs.Pose pose_) {
        setPosition(pose_.getPosition());
        setOrientation(pose_.getOrientation());
    }

    @Override
    public Point getPosition() {
        return position;
    }

    @Override
    public void setPosition(geometry_msgs.Point point) {
        this.position = new william.chamberlain.androidvosopencvros.device.Point(point);
    }

    public void setPosition(william.chamberlain.androidvosopencvros.device.Point point_) {
        this.position = point_;
    }


    @Override
    public Quaternion getOrientation() {
        return orientation;
    }

    @Override
    public void setOrientation(geometry_msgs.Quaternion quaternion) {
        this.orientation = new william.chamberlain.androidvosopencvros.device.Quaternion(quaternion);
    }

    public void setOrientation(william.chamberlain.androidvosopencvros.device.Quaternion quaternion_) {
        this.orientation = quaternion_;
    }

    @Override
    public RawMessage toRawMessage() {
        return null;
    }

    @Override
    public String toString() {
        return "Pose{" +
                "position=" + position +
                ", orientation=" + orientation +
                '}';
    }
}
