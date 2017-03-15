package william.chamberlain.androidvosopencvros;

import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import geometry_msgs.Point;
import geometry_msgs.Quaternion;
import vos_aa1.LocaliseFromAFeature;
import vos_aa1.LocaliseFromAFeatureRequest;
import vos_aa1.LocaliseFromAFeatureResponse;

/**
 * Created by will on 15/03/17.
 */

public class LocaliseFromAFeatureServer extends AbstractNodeMain {
    private ServiceServer<LocaliseFromAFeatureRequest, LocaliseFromAFeatureResponse> service;
    private String nodeNamespace;
    private PosedEntity posedEntity;
    private DetectedFeaturesHolder detectedFeaturesHolder;


    public void setNodeNamespace(java.lang.String nodeNamespace) {
        this.nodeNamespace = nodeNamespace;
    }

    public void setPosedEntity(PosedEntity posedEntity) {
        this.posedEntity = posedEntity;
    }

    public void setDetectedFeaturesHolder(DetectedFeaturesHolder detectedFeaturesHolder) {
        this.detectedFeaturesHolder = detectedFeaturesHolder;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("androidvosopencvros/localise_from_a_feature");
    }

    @Override
    public void onStart(ConnectedNode connectedNode_) {
        service = connectedNode_.newServiceServer( nodeNamespace+"/localise_from_a_feature", LocaliseFromAFeature._TYPE,
            new ServiceResponseBuilder<LocaliseFromAFeatureRequest,LocaliseFromAFeatureResponse>() {

                @Override
                public void build(LocaliseFromAFeatureRequest request, LocaliseFromAFeatureResponse response)
                        throws ServiceException {
                    response.setPoseFound(false);
                    if(!posedEntity.poseKnown()) {
                        return;
                    }
                    for (DetectedFeature detectedFeature: detectedFeaturesHolder.detectedFeatureList()) {
                        if (detectedFeature.algorithm.equals(request.getVisualFeature().getAlgorithm())
                            && detectedFeature.descriptor.equals(Integer.toString( request.getVisualFeature().getId()) ) ) {

                            // from-this_camera-to-tag
                            Transform from_this_camera_to_tag = new Transform(detectedFeature.translation_to_tag_in_robot_convention,detectedFeature.quaternion_rotation_to_tag);

                            // from-world-to-thiscamera
                            double[] world_to_thiscamera_translation = posedEntity.getPosition();
                            double[] world_to_thiscamera_quaternion  = posedEntity.getOrientation();
                            org.ros.rosjava_geometry.Vector3 camera_translation_from_world_to_thiscamera
                                    = new org.ros.rosjava_geometry.Vector3(world_to_thiscamera_translation[0],world_to_thiscamera_translation[1],world_to_thiscamera_translation[2]);
                            org.ros.rosjava_geometry.Quaternion camera_quaternion_from_world_to_thiscamera
                                    = new org.ros.rosjava_geometry.Quaternion(world_to_thiscamera_quaternion[0],world_to_thiscamera_quaternion[1],world_to_thiscamera_quaternion[2],world_to_thiscamera_quaternion[3]);
                            org.ros.rosjava_geometry.Transform from_world_to_thiscamera
                                    = new org.ros.rosjava_geometry.Transform(camera_translation_from_world_to_thiscamera,camera_quaternion_from_world_to_thiscamera);

                            //apply from camera-to-tag to from-world-to-camera
                            Transform from_world_to_tag = from_world_to_thiscamera.multiply(from_this_camera_to_tag);
                            // now know (1) this camera's pose in the world (2) the pose of the requested tag from this camera (3) therefore the requested tag in the world via this camera's observations

                            // from requestor-to-tag

                            // rotate the tag frame 180 around z, so it faces away
//                            org.ros.rosjava_geometry.Quaternion  q_tmp180x = new org.ros.rosjava_geometry.Quaternion(1.0,0.0,0.0,0.0);
//                            org.ros.rosjava_geometry.Vector3    v3_tmp180x = new org.ros.rosjava_geometry.Vector3(0.0,0.0,0.0);
//                            Transform tmp180x                                 = new Transform(v3_tmp180x,q_tmp180x);
//                            Transform from_tag_to_req_tmp180x   = from_world_to_tag.multiply(tmp180x);

                            // apply the inverse of requestor-to-tag quaternion, to point back toward the requestor camera
                            Quaternion q                                    = request.getVisualFeature().getPose().getPose().getOrientation();  // fix the coordinate system convention for this, as MainActivity line 654 : (qz,-qx,-qy,qw);
                            org.ros.rosjava_geometry.Quaternion q_of_q      = new org.ros.rosjava_geometry.Quaternion(q.getZ(),-q.getX(),-q.getY(),q.getW());
                            org.ros.rosjava_geometry.Vector3 v3_no_change   = new Vector3(0.0, 0.0, 0.0);
                            Transform tmp                                   = new Transform(v3_no_change,q_of_q.invert());
                            Transform from_tag_to_req_tmp;
//                            from_tag_to_req_tmp                 = from_tag_to_req_tmp180x.multiply(tmp);
                            from_tag_to_req_tmp                 = from_world_to_tag.multiply(tmp);

                            org.ros.rosjava_geometry.Quaternion q_no_change = new org.ros.rosjava_geometry.Quaternion(0.0, 0.0, 0.0, 1.0);
                            Point p                                         = request.getVisualFeature().getPose().getPose().getPosition();
                            org.ros.rosjava_geometry.Vector3 v3_of_p        = new org.ros.rosjava_geometry.Vector3(-p.getX(), -p.getY(), -p.getZ());
                            Transform t                                     = new Transform(v3_of_p, q_no_change);
                            Transform from_tag_to_req           = from_tag_to_req_tmp.multiply(t);
//                            Transform from_tag_to_requestor = from_requestor_to_tag.invert();

//                            Transform from_world_to_requestor = from_world_to_tag.multiply(from_tag_to_requestor);

                            response.setPoseFound(true);
                            response.setPose(from_tag_to_req.toPoseMessage(response.getPose()));
                        }
                    }
                }

            }
            );
    }


}
