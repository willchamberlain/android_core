package william.chamberlain.androidvosopencvros.device;


import org.ros.internal.message.RawMessage;

/**
 * Created by will on 22/03/17.
 */

public class Quaternion implements geometry_msgs.Quaternion {
    public Quaternion() {
        super();
    }

    public Quaternion(geometry_msgs.Quaternion quaternion_) {
        this.x = quaternion_.getX();
        this.y = quaternion_.getY();
        this.z = quaternion_.getZ();
        this.w = quaternion_.getW();
    }

    @Override
    public int hashCode() {
        return super.hashCode();
    }

    @Override
    public boolean equals(Object obj) {
        return super.equals(obj);
    }

    @Override
    protected Object clone() throws CloneNotSupportedException {
        return super.clone();
    }

    @Override
    protected void finalize() throws Throwable {
        super.finalize();
    }

    double x;
    double y;
    double z;
    double w;

    @Override
    public double getX() {
        return x;
    }

    @Override
    public void setX(double v) {
        x = v;
    }

    @Override
    public double getY() {
        return y;
    }

    @Override
    public void setY(double v) {
        y = v;
    }

    @Override
    public double getZ() {
        return z;
    }

    @Override
    public void setZ(double v) {
        z = v;
    }

    @Override
    public double getW() {
        return w;
    }

    @Override
    public void setW(double v) {
        w = v;
    }

    @Override
    public RawMessage toRawMessage() {
        return null;
    }

    public Quaternion(double x, double y, double z, double w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    @Override
    public String toString() {
        return "Quaternion{" +
                "x=" + x +
                ", y=" + y +
                ", z=" + z +
                ", w=" + w +
                '}';
    }
}
