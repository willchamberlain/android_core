package william.chamberlain.androidvosopencvros;

import geometry_msgs.Point;
import geometry_msgs.Quaternion;

/**
 * Created by will on 22/02/17.
 */

public class Geometry {

    public static Point applyTranslationParams(double x, double y, double z, Point translationToFeature) {
        translationToFeature.setX(x);
        translationToFeature.setY(y);
        translationToFeature.setZ(z);
        return translationToFeature;
    }

    public static Quaternion applyQuaternionParams(double qx, double qy, double qz, double qw, Quaternion featureOrientation) {
        featureOrientation.setX(qx);
        featureOrientation.setY(qy);
        featureOrientation.setZ(qz);
        featureOrientation.setW(qw);
        return featureOrientation;
    }
}
