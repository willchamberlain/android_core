package william.chamberlain.androidvosopencvros.ros_types;


import org.ros.message.Time;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;

/**
 * Created by will on 17/11/17.
 */

public class RosTypes {
    Transform copyTransform(Transform transform_) {
        Vector3 translation = copyVector3(transform_);
        Quaternion rotation = copyQuaternion(transform_);
        return new Transform(translation,rotation);
    }

    public static Vector3 copyVector3(Transform transform_) {
        return new Vector3(transform_.getTranslation().getX(),transform_.getTranslation().getY(),transform_.getTranslation().getZ());
    }

    public static Quaternion copyQuaternion(Transform transform_) {
        return new Quaternion(transform_.getRotationAndScale().getX(),transform_.getRotationAndScale().getY(),transform_.getRotationAndScale().getZ(),transform_.getRotationAndScale().getW());
    }

    /******/


    public static void header(PoseStamped poseStamped, String frameId_) {
        std_msgs.Header header = poseStamped.getHeader();
        long time_delta_millis = System.currentTimeMillis();
        header.setStamp(Time.fromMillis(time_delta_millis));
        header.setFrameId(frameId_);
    }


    public static void updatePoseStampedFromTransform(Transform transform_, PoseStamped poseStamped) {
        Pose pose = poseStamped.getPose();
        updatePoseFromTransform(transform_, pose);
    }

    public static void updatePoseFromTransform(Transform transform_, Pose pose) {
        Point point = pose.getPosition();
        geometry_msgs.Quaternion quaternion = pose.getOrientation();
        updatePoseFromTransform(transform_, quaternion);
        updatePoseFromTransform(transform_, point);
    }

    public static void updatePoseFromTransform(Transform transform_, geometry_msgs.Quaternion quaternion) {
        quaternion.setX(transform_.getRotationAndScale().getX());
        quaternion.setY(transform_.getRotationAndScale().getY());
        quaternion.setZ(transform_.getRotationAndScale().getZ());
        quaternion.setW(transform_.getRotationAndScale().getW());
    }

    public static void updatePoseFromTransform(Transform transform_, Point point) {
        point.setX(transform_.getTranslation().getX());
        point.setY(transform_.getTranslation().getY());
        point.setZ(transform_.getTranslation().getZ());
    }

}
