package william.chamberlain.androidvosopencvros;

import junit.framework.TestCase;

//import org.junit.After;
//import org.junit.Before;
//import org.junit.Test;

/**
 * Created by will on 7/04/17.
 */
public class DataTrackTest extends TestCase {
    static boolean did_setUp_run = false;

//    @Before
    public void setUp() throws Exception {
        if(did_setUp_run) {
            throw new RuntimeException("tearDown() should have reset did_setUp_run to false.");
        }
        did_setUp_run = true;
    }

//    @After
    public void tearDown() throws Exception {
        did_setUp_run = false;
    }

//    @Test
    public void test_add() throws Exception {
        int size_ = 10;
        DataTrack dataTrack = new DataTrack(size_);
        assertEquals("Have added 0 DetectedFeatures: data track should have 0 elements.", 0 , dataTrack.data().length );
        org.ros.rosjava_geometry.Vector3 translation = new org.ros.rosjava_geometry.Vector3(1,2,3);
        org.ros.rosjava_geometry.Quaternion rotation = new org.ros.rosjava_geometry.Quaternion(0,0,0,1);
        DetectedFeature detectedFeature = new DetectedFeature("thisAlg","someId",translation, rotation);
        dataTrack.add(detectedFeature);
        assertEquals("Have added 1 DetectedFeature: data track should have 1 element.", 1 , dataTrack.data().length );
        for(int i_=0;i_<50;i_++) {
            dataTrack.add(detectedFeature);
        }
        assertEquals("Have added 51 DetectedFeatures: data track should have "+size_+" elements.", size_ , dataTrack.data().length );
    }

//    @org.junit.Test
    public void test_setUp_runs() throws Exception {
        assertTrue("setUp() should have run", did_setUp_run);
    }

}