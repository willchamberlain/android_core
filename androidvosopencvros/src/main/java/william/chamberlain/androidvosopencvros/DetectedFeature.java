package william.chamberlain.androidvosopencvros;

import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Vector3;

/**
 * Convention: from the observer frame, apply the translation to get to the feature frame origin, then apply the rotation to get the full feature frame pose.
 *
 * Created by will on 15/03/17.
 */

public class DetectedFeature {
    String algorithm;
    String descriptor;
    org.ros.rosjava_geometry.Vector3 translation_to_tag_in_robot_convention;
    org.ros.rosjava_geometry.Quaternion quaternion_rotation_to_tag;

    public DetectedFeature(String algorithm, String descriptor, Vector3 translation_to_tag_in_robot_convention, Quaternion quaternion_rotation_to_tag) {
        this.algorithm = algorithm;
        this.descriptor = descriptor;
        this.translation_to_tag_in_robot_convention = translation_to_tag_in_robot_convention;
        this.quaternion_rotation_to_tag = quaternion_rotation_to_tag;
    }

    public String featureCanonicalDescriptor() {
        return Descriptor.featureCanonicalDescriptor(this);
    }
}
