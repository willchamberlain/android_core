package william.chamberlain.androidvosopencvros;

import org.ros.internal.message.RawMessage;

import geometry_msgs.Pose;

/**
 * Created by will on 20/07/17.
 */
class WhereIsAsPubLocal {
    private String algorithm, descriptor, requestId, returnUrl;
    private Pose relationToBase;
    private int rate;

    public String getAlgorithm() {
        return algorithm;
    }

    public void setAlgorithm(String value) {
        algorithm = value;
    }

    public String getDescriptor() {
        return descriptor;
    }

    public void setDescriptor(String value) {
        descriptor = value;
    }

    public String getRequestId() {
        return requestId;
    }

    public void setRequestId(String value) {
        requestId = value;
    }

    public int getRate() {
        return rate;
    }

    public void setRate(int value) {
        rate = value;
    }

    public Pose getRelationToBase() {
        return relationToBase;
    }

    public void setRelationToBase(Pose value) {
        relationToBase = value;
    }

    public String getReturnUrl() {
        return null;
    }

    public void setReturnUrl(String value) {
        returnUrl = value;
    }

    public RawMessage toRawMessage() {
        return null;
    }

    public WhereIsAsPubLocal(String algorithm, String descriptor, int rate) {
        this.algorithm = algorithm;
        this.descriptor = descriptor;
        this.rate = rate;
    }

    public WhereIsAsPubLocal(String algorithm, String descriptor, int rate, double x, double y, double z, double qx, double qy, double qz, double qw) {
        this.algorithm = algorithm;
        this.descriptor = descriptor;
        this.rate = rate;
        this.relationToBase =
                new william.chamberlain.androidvosopencvros.device.Pose(
                    new william.chamberlain.androidvosopencvros.device.Point(x,y,z),
                    new william.chamberlain.androidvosopencvros.device.Quaternion(qx,qy,qz,qw) );
    }
}
