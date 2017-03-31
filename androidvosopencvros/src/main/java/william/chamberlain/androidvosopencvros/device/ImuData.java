package william.chamberlain.androidvosopencvros.device;

import org.ros.internal.message.RawMessage;

import geometry_msgs.Quaternion;
import geometry_msgs.Vector3;
import sensor_msgs.Imu;
import std_msgs.Header;

/**
 * Created by will on 22/03/17.
 */

public class ImuData implements Imu {
    @Override
    public Header getHeader() {
        return null;
    }

    @Override
    public void setHeader(Header header) {

    }

    @Override
    public Quaternion getOrientation() {
        return null;
    }

    @Override
    public void setOrientation(Quaternion quaternion) {

    }

    @Override
    public double[] getOrientationCovariance() {
        return new double[0];
    }

    @Override
    public void setOrientationCovariance(double[] doubles) {

    }

    @Override
    public Vector3 getAngularVelocity() {
        return null;
    }

    @Override
    public void setAngularVelocity(Vector3 vector3) {

    }

    double[] angularVelCovar = new double[0];

    @Override
    public double[] getAngularVelocityCovariance() {
        return angularVelCovar;
    }

    @Override
    public void setAngularVelocityCovariance(double[] doubles) {
        angularVelCovar = doubles;
    }

    william.chamberlain.androidvosopencvros.device.Vector3 linearAcceleration;
    
    @Override
    public Vector3 getLinearAcceleration() {
        return linearAcceleration;
    }

    @Override
    public void setLinearAcceleration(Vector3 vector3) {
        linearAcceleration = (william.chamberlain.androidvosopencvros.device.Vector3)vector3;
    }

    double[] linearCovar = new double[0];
    @Override
    public double[] getLinearAccelerationCovariance() {
        return linearCovar;
    }

    @Override
    public void setLinearAccelerationCovariance(double[] doubles) {
        linearCovar = doubles;
    }

    @Override
    public RawMessage toRawMessage() {
        return null;
    }
}
