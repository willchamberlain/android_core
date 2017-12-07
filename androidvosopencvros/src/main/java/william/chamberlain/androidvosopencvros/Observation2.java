package william.chamberlain.androidvosopencvros;

import org.ejml.data.DenseMatrix64F;
import org.ros.rosjava_geometry.Transform;

import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Quaternion_F64;

/**
 * Created by will on 24/11/17.
 */

public class Observation2 {
    String robotId;
    java.util.Date imageCaptureTime;
    PixelPosition pixelPosition;
    Transform map_to_baselink_pose;
    Se3_F64 baselink_to_tag_transform;
    Se3_F64 map_to_tag_transform_boofcv;
    public Observation2(String robotId_, java.util.Date imageCaptureTime_, PixelPosition pixelPosition_, Transform map_to_baselink_pose_, Se3_F64 baselink_to_tag_transform_) {
        this.robotId            =   robotId_;
        this.imageCaptureTime   =   imageCaptureTime_;
        this.pixelPosition      =   pixelPosition_;
        this.map_to_baselink_pose      = map_to_baselink_pose_;
        this.baselink_to_tag_transform = baselink_to_tag_transform_;

        Se3_F64 mtbp_transform_boofcv = new Se3_F64();
        mtbp_transform_boofcv.setTranslation(
                map_to_baselink_pose.getTranslation().getX(), map_to_baselink_pose.getTranslation().getY(),
                map_to_baselink_pose.getTranslation().getZ());
        Quaternion_F64 mtbp_quat_boofcv = new Quaternion_F64(
                map_to_baselink_pose.getRotationAndScale().getW(), map_to_baselink_pose.getRotationAndScale().getX(),
                map_to_baselink_pose.getRotationAndScale().getY(), map_to_baselink_pose.getRotationAndScale().getZ());
        DenseMatrix64F mtbp_rotmat_boofcv = new DenseMatrix64F(3, 3);
        mtbp_transform_boofcv.setRotation(ConvertRotation3D_F64.quaternionToMatrix(mtbp_quat_boofcv, mtbp_rotmat_boofcv));

        // NOTE: boofcv does this in reverse order from normal matrix mult notation
        this.map_to_tag_transform_boofcv = new Se3_F64();
        this.baselink_to_tag_transform.concat(mtbp_transform_boofcv,this.map_to_tag_transform_boofcv);
        Quaternion_F64 map_to_tag_quaterion = new Quaternion_F64();
        ConvertRotation3D_F64.matrixToQuaternion(map_to_tag_transform_boofcv.getRotation(),map_to_tag_quaterion);
        System.out.println("new Observation2:\\n\\t baselink_to_tag_transform="+baselink_to_tag_transform.toString()+",\\n\\tmap_to_tag_transform_boofcv="+map_to_tag_transform_boofcv+" ,\\n\\t map_to_tag_transform_boofcv quaternion = "+map_to_tag_quaterion.toString()+",\\n\\t"+toString());

        // to get the world point observed, convert this.pose into BoofCV transform matrix via quaternion and translation
        //   transformOfFeatureInVisualModel.setTranslation(position.getX(), position.getY(), position.getZ());
        //   Quaternion_F64(double w, double x, double y, double z) / Quaternion_F64 rotationOfFeatureInVisualModel_q = convertRosToBoofcvQuaternion(visionTask);
        //   transformOfFeatureInVisualModel.setRotation(
        //          ConvertRotation3D_F64.quaternionToMatrix(rotationOfFeatureInVisualModel_q, rotationOfFeatureInVisualModel_m));
        //
    }

    @Override
    public String toString() {
        Quaternion_F64 map_to_base_link_quaterion = new Quaternion_F64(
                map_to_baselink_pose.getRotationAndScale().getW(), map_to_baselink_pose.getRotationAndScale().getX(),
                map_to_baselink_pose.getRotationAndScale().getY(), map_to_baselink_pose.getRotationAndScale().getZ());
        Quaternion_F64 map_to_tag_quaterion = new Quaternion_F64();
        ConvertRotation3D_F64.matrixToQuaternion(map_to_tag_transform_boofcv.getRotation(),map_to_tag_quaterion);
        return "Observation2{" +
                " robotId=" + robotId +
                ", framteTime=" + imageCaptureTime +
                ", pixelPosition=" + pixelPosition +
                ", map_to_baselink_pose=" + map_to_baselink_pose +
                ", map_to_baselink_pose quaternion=" + map_to_base_link_quaterion +
                ", baselink_to_tag_transform=" + baselink_to_tag_transform +
                ", map_to_tag_transform_boofcv=" + map_to_tag_transform_boofcv +
                ", map_to_tag_transform_boofcv quaternion=" + map_to_tag_quaterion +
                '}';
    }
}