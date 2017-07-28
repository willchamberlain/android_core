package william_chamberlain_examples.localisation;

import boofcv.alg.fiducial.square.QuadPoseEstimator;
import boofcv.alg.geo.WorldToCameraToPixel;

/**
 * Proof of concept for P3P with BoofCV: given sparse features/objects/occlusions/markers in the
 * environment, estimate the camera pose given 4 known 3d points and the 4 corresponding 2d points
 * in the image plane - i.e. the 4 2d pixel locations in the image.
 *
 * Created by will on 21/07/17.
 */

public class P3PwithBoofCV {


    public static void main(String[] args) {
        System.out.println("Starting 1 ...");
        System.out.println("... Starting 2 ...");
        System.out.println("... Starting 3.");

        QuadPoseEstimator estimator = new QuadPoseEstimator(1e-8,200);
        estimator.setFiducial(-0.5,0.5,0.5,0.5,0.5,-0.5,-0.5,-0.5);



        throw new RuntimeException("it worked");
    }
}
