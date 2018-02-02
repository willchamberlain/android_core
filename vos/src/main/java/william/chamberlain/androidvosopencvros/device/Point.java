package william.chamberlain.androidvosopencvros.device;

import org.ros.internal.message.RawMessage;

/**
 * Created by will on 31/08/17.
 */

public class Point implements geometry_msgs.Point {

    private double x,y,z;

    public Point(double x_, double y_, double z_) {
        this.x=x_;
        this.y=y_;
        this.z=z_;
    }

    public Point(geometry_msgs.Point point_) {
        this.x = point_.getX();
        this.y = point_.getY();
        this.z = point_.getZ();
    }

    @Override
    public double getX() {
        return 0;
    }

    @Override
    public void setX(double v) {
        this.x = v;
    }

    @Override
    public double getY() {
        return y;
    }

    @Override
    public void setY(double v) {
        this.y = v;
    }

    @Override
    public double getZ() {
        return z;
    }

    @Override
    public void setZ(double v) {
        this.z = v;
    }

    @Override
    public RawMessage toRawMessage() {
        return null;
    }

    @Override
    public String toString() {
        return "Point{" +
                "x=" + x +
                ", y=" + y +
                ", z=" + z +
                '}';
    }
}
