/**
 * Based on boofcv.examples.sfm.ExamplePnP.
 *
 * -----------------------------
 * BoofCV license of ExamplePnP:
 * -----------------------------
 * [Copyright (c) 2011-2017, Peter Abeles. All Rights Reserved.]
 *
 * This file is part of BoofCV (http://boofcv.org).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * ---------------------------------
 * End BoofCV license of ExamplePnP.
 * ---------------------------------
 */

package william.chamberlain.androidvosopencvros;

import org.ddogleg.fitting.modelset.ransac.Ransac;

import java.util.ArrayList;
import java.util.List;

import boofcv.abst.geo.Estimate1ofPnP;
import boofcv.abst.geo.RefinePnP;
import boofcv.alg.distort.LensDistortionOps;
import boofcv.factory.geo.ConfigPnP;
import boofcv.factory.geo.ConfigRansac;
import boofcv.factory.geo.EnumPNP;
import boofcv.factory.geo.FactoryMultiView;
import boofcv.factory.geo.FactoryMultiViewRobust;
import boofcv.struct.calib.CameraPinholeRadial;
import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.geo.Point2D3D;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;

/**
 * Created by will on 24/07/17.
 */

public class MultiFeature {

    CameraPinholeRadial intrinsic;

    public MultiFeature(CameraPinholeRadial intrinsic_) {
        this.intrinsic = intrinsic_;
    }

    /** ****************************************
     image:                                                   /mnt/nixbig/ownCloud/project_AA1__1_1/results/2017_07_23_P3P_test/IMG_20170723_212755.jpg
     measured world coordinates (Z=0.0000):  	                /mnt/nixbig/ownCloud/project_AA1__1_1/results/2017_07_23_P3P_test/data.txt
     measured pixel coordinates in the undistorted image: :  	/mnt/nixbig/ownCloud/project_AA1__1_1/results/2017_07_23_P3P_test/data.txt

     camera calibration:
     model: pinhole_radial_tangential
     pinhole:
     height: 3120
     fx: 3497.344137205399
     fy: 3498.654679053389
     width: 4160
     cy: 1517.8452462318207
     cx: 2090.8667940953733
     skew: 0.0
     radial_tangential:
     t2: 0.0
     t1: 0.0
     radial:
     - 0.0673437541514291
     - -0.1418426025730499
     */
    public Se3_F64 estimateCameraPoseFromPixelsAndMeasurements(int numPointsToUse) {

        // Compute a single solution using EPNP
        // 10 iterations is what JavaDoc recommends, but might need to be tuned.
        // 0 test points.  This parameters is actually ignored because EPNP only returns a single solution
//		Estimate1ofPnP pnp = FactoryMultiView.computePnP_1(EnumPNP.EPNP, 10, 0);
//		Se3_F64 worldToCamera = new Se3_F64();
//		pnp.process(observations,worldToCamera);

//      CameraPinholeRadial(double fx, double fy, double skew, double cx, double cy, int width, int height)
        CameraPinholeRadial cameraIntrinsics = new CameraPinholeRadial(
                3497.344137205399,3498.654679053389,
                0,
                2090.8667940953733,1517.8452462318207,
                4160,3120).fsetRadial(0.0673437541514291,-0.1418426025730499);

        List<Point2D3D> observations = new ArrayList<Point2D3D>();

        // transform from pixel coordinates to normalized pixel coordinates, which removes lens distortion
        // note: rectified/undistorted AND normalised to range -1:1 - see http://answers.opencv.org/question/83807/normalized-camera-image-coordinates/
//        Point2Transform2_F64 pixelToNormAndRectTrans = LensDistortionOps.narrow(cameraIntrinsics).undistort_F64(true,false);
        Point2Transform2_F64 pixelToNormAndRectTrans = LensDistortionOps.transformPoint(cameraIntrinsics).undistort_F64(true,false);
        Point2D_F64 normalisedAndRectPoint = new Point2D_F64();
        double x,y,z;
        z=0;
        double x_pixel,y_pixel;

        // add each point
        /*
worldpoints =
        1280        1920        1920        1280        1280        -640        -640        3200        3200         320        -960
        1920        1920        2240        2240        3200        3200        3520        3520        4160        4160        4160
pixels
2839.60714285714	3518.62500000000	3278.35714285714	2648.08928571429	2240.67857142857	332.464285714285	377.732142857142	3473.35714285714	3184.33928571429	1265.67857142857
2700.90178571429	2526.79464285714	2331.79464285714	2478.04464285714	2000.99107142857	2345.72321428571	2182.06250000000	1666.70535714286	1527.41964285714	1816.43750000000
        */
        /*
        double[] worldX  = {1280,               1920,               1920,               1280,               1280,               -640,               -640,               3200,               3200,               320};
        double[] worldY  = {1920,               1920,               2240,               2240,               3200,               3200,               3520,               3520,               4160,               4160};
        double[] pixelsX = {2839.60714285714,	3518.62500000000,	3278.35714285714,	2648.08928571429,	2240.67857142857,	332.464285714285,	377.732142857142,	3473.35714285714,	3184.33928571429,	1265.67857142857};
        double[] pixelsY = {2700.90178571429,	2526.79464285714,	2331.79464285714,	2478.04464285714,	2000.99107142857,	2345.72321428571,	2182.06250000000,	1666.70535714286,	1527.41964285714,	1816.43750000000}; */

        // reduced set: check whether is stable for smaller baseline
        double[] worldX  = {1280,               1920,               1920,               1280};
        double[] worldY  = {1920,               1920,               2240,               2240};
        double[] pixelsX = {2839.60714285714,	3518.62500000000,	3278.35714285714,	2648.08928571429};
        double[] pixelsY = {2700.90178571429,	2526.79464285714,	2331.79464285714,	2478.04464285714};

        for (int i_ = 0; i_ < worldX.length && i_< numPointsToUse; i_++) {
            x=worldX[i_];          y=worldY[i_];               z=0;  // world 3d coordinates
            x_pixel=pixelsX[i_];   y_pixel=pixelsY[i_];
            pixelToNormAndRectTrans.compute(x_pixel,y_pixel,normalisedAndRectPoint); //rectify and normalise
            addToObservations( x, y, z, normalisedAndRectPoint,  observations ); // Save the observation
        }
        // get the PnP estimate
        Se3_F64 refinedWorldToCamera = estimateNoOutliers(observations);
        return refinedWorldToCamera;
    }

    /** ****************************************
     */
    public void addToObservations(double x, double y, double z, Point2D_F64 normalisedAndRectifiedImagePoint, List<Point2D3D> observations ) {
        // Save the observation
        Point3D_F64 worldPt = new Point3D_F64(x,y,z);
        Point2D3D point2dAnd3d = new Point2D3D();
        point2dAnd3d.getLocation().set(worldPt);
        point2dAnd3d.getObservation().set(normalisedAndRectifiedImagePoint.x,normalisedAndRectifiedImagePoint.y);

        observations.add(point2dAnd3d);
    }


    /** ****************************************
     * Assumes all observations actually match the correct/real 3D point
     */
    public Se3_F64 estimateNoOutliers( List<Point2D3D> observations ) {

        // Compute a single solution using EPNP
        // 10 iterations is what JavaDoc recommends, but might need to be tuned.
        // 0 test points.  This parameters is actually ignored because EPNP only returns a single solution
        Estimate1ofPnP pnp = FactoryMultiView.computePnP_1(EnumPNP.EPNP, 10, 0);

        Se3_F64 worldToCamera = new Se3_F64();
        pnp.process(observations,worldToCamera);

        // For some applications the EPNP solution might be good enough, but let's refine it
        RefinePnP refine = FactoryMultiView.refinePnP(1e-8,200);

        Se3_F64 refinedWorldToCamera = new Se3_F64();

        if( !refine.fitModel(observations,worldToCamera,refinedWorldToCamera) )
            throw new RuntimeException("Refined failed! Input probably bad...");

        return refinedWorldToCamera;
    }

    /**
     * Uses robust techniques to remove outliers
     */
    public Se3_F64 estimateOutliers( List<Point2D3D> observations) {
        // We can no longer trust that each point is a real observation.  Let's use RANSAC to separate the points
        // You will need to tune the number of iterations and inlier threshold!!!
        Ransac<Se3_F64,Point2D3D> ransac =
                FactoryMultiViewRobust.pnpRansac(new ConfigPnP(intrinsic),new ConfigRansac(300,1.0));

        // Observations must be in normalized image coordinates!  See javadoc of pnpRansac
        if( !ransac.process(observations) )
            throw new RuntimeException("Probably got bad input data with NaN inside of it");

        System.out.println("Inlier size "+ransac.getMatchSet().size());
        Se3_F64 worldToCamera = ransac.getModelParameters();

        // You will most likely want to refine this solution too.  Can make a difference with real world data
        RefinePnP refine = FactoryMultiView.refinePnP(1e-8,200);

        Se3_F64 refinedWorldToCamera = new Se3_F64();

        // notice that only the match set was passed in
        if( !refine.fitModel(ransac.getMatchSet(),worldToCamera,refinedWorldToCamera) )
            throw new RuntimeException("Refined failed! Input probably bad...");

        return refinedWorldToCamera;
    }

}
