package william_chamberlain_examples.localisation;

import boofcv.alg.fiducial.square.QuadPoseEstimator;
import boofcv.alg.geo.WorldToCameraToPixel;

/**
 * See /mnt/nixbig/downloads/boofcv_and_related/lessthanoptimal/BoofCV/examples/src/boofcv/examples/sfm/ExamplePnP.java
 *
 *
 * Proof of concept for P3P with BoofCV: given sparse features/objects/occlusions/markers in the
 * environment, estimate the camera pose given 4 known 3d points and the 4 corresponding 2d points
 * in the image plane - i.e. the 4 2d pixel locations in the image.
 */

public class P3PwithBoofCV {


    public static void main(String[] args) {
//  see /mnt/nixbig/downloads/boofcv_and_related/lessthanoptimal/BoofCV/examples/src/boofcv/examples/sfm/ExamplePnP.java
//        System.out.println("Starting 1 ...");
//        System.out.println("... Starting 2 ...");
//        System.out.println("... Starting 3.");
//
//        QuadPoseEstimator estimator = new QuadPoseEstimator(1e-8,200);
//        estimator.setFiducial(-0.5,0.5,0.5,0.5,0.5,-0.5,-0.5,-0.5);
//
//
//
//        throw new RuntimeException("it worked");
    }
}
