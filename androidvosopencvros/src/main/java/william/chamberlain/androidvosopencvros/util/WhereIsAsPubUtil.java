package william.chamberlain.androidvosopencvros.util;

import vos_aa1.WhereIsAsPub;

/**
 * Created by will on 17/07/17.
 */

public class WhereIsAsPubUtil {
    public static String toString(WhereIsAsPub message) {
        return "WhereIsAsPub{" +
                " robotId='" + message.getRobotId() + '\'' +
                ", algorithm='" + message.getAlgorithm() + '\'' +
                ", descriptor='" + message.getDescriptor() + '\'' +
                ", requestId='" + message.getRequestId() + '\'' +
                ", relationToBase=" + message.getRelationToBase() +
                ", returnUrl='" + message.getReturnUrl() + '\'' +
                ", rate=" + message.getRate() +
                ", relationToBase=" + message.getRelationToBase() +
                '}';
    }
}
