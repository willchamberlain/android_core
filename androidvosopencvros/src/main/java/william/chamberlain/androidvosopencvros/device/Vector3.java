package william.chamberlain.androidvosopencvros.device;


import org.ros.internal.message.RawMessage;

/**
 * Created by will on 22/03/17.
 */

public class Vector3 implements geometry_msgs.Vector3 {

    double x;
    double y;
    double z;

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
    public RawMessage toRawMessage() {
        return null;
    }
}
