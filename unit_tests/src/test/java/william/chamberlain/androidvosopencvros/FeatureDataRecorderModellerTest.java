package william.chamberlain.androidvosopencvros;
//
//import org.junit.After;
//import org.junit.Before;
//import org.junit.Test;

import static org.junit.Assert.*;

/**
 * Created by will on 7/04/17.
 */
public class FeatureDataRecorderModellerTest {
//    @Before
    public void setUp() throws Exception {

    }

//    @After
    public void tearDown() throws Exception {

    }

//    @Test
    public void test_addDetection() throws Exception {
        FeatureDataRecorderModeller fdrm_to_test = new FeatureDataRecorderModeller();
        org.ros.rosjava_geometry.Vector3 translation;
        org.ros.rosjava_geometry.Quaternion rotation;
        DetectedFeature detectedFeature;
        DetectedFeature averaged_detectedFeature_1;
        DetectedFeature averaged_detectedFeature_2;
        translation = new org.ros.rosjava_geometry.Vector3(1,2,3);
        rotation = new org.ros.rosjava_geometry.Quaternion(0,0,0,1);
        detectedFeature = new DetectedFeature("thisAlg","someId",translation, rotation);
        fdrm_to_test.addDetection(detectedFeature);
        translation = new org.ros.rosjava_geometry.Vector3(2,3,4);
        rotation = new org.ros.rosjava_geometry.Quaternion(0,0,1,0);
        detectedFeature = new DetectedFeature("thisAlg","someId",translation, rotation);
        fdrm_to_test.addDetection(detectedFeature);
        translation = new org.ros.rosjava_geometry.Vector3(3,4,5);
        rotation = new org.ros.rosjava_geometry.Quaternion(0,1,0,0);
        detectedFeature = new DetectedFeature("thisAlg","someId",translation, rotation);
        fdrm_to_test.addDetection(detectedFeature);
        translation = new org.ros.rosjava_geometry.Vector3(4,5,6);
        rotation = new org.ros.rosjava_geometry.Quaternion(1,0,0,0);
        detectedFeature = new DetectedFeature("thisAlg","someId",translation, rotation);
        fdrm_to_test.addDetection(detectedFeature);
        averaged_detectedFeature_1 = fdrm_to_test.average("thisAlg","someId");
    }

}