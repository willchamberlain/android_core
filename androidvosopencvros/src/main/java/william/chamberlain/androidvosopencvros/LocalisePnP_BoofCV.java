
/*
 * Copyright (c) 2011-2017, Peter Abeles. All Rights Reserved.
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
 */


package william.chamberlain.androidvosopencvros;


        import boofcv.abst.geo.Estimate1ofPnP;
        import boofcv.abst.geo.RefinePnP;
        import boofcv.alg.distort.LensDistortionOps;
        import boofcv.factory.geo.*;
        import boofcv.struct.calib.CameraPinholeRadial;
        import boofcv.struct.distort.Point2Transform2_F64;
        import boofcv.struct.geo.Point2D3D;
        import georegression.struct.point.Point2D_F64;
        import georegression.struct.point.Point3D_F64;
        import georegression.struct.se.Se3_F64;

        import org.ddogleg.fitting.modelset.ransac.Ransac;


        import georegression.struct.point.Vector3D_F64;
//        import org.ejml.data.DenseMatrix64F;
//        import org.ejml.ops.CommonOps;

        import java.util.ArrayList;
        import java.util.List;
        import java.util.Random;
public class LocalisePnP_BoofCV implements PoseFrom3D2DPointMatches {

    // describes the intrinsic camera parameters.
    CameraPinholeRadial intrinsic = new CameraPinholeRadial(500,490,0,320,240,640,480).fsetRadial(0.1,-0.05);

    // Used to generate random observations
    Random rand = new Random(234);
    public static void main(String[] args) {
        // test #2 : reduced set: check whether is stable for smaller baseline.
        double[] worldX_test  = {1280,               1920,               1920,               1280};
        double[] worldY_test  = {1920,               1920,               2240,               2240};
        double[] pixelsX_test = {2839.60714285714,	3518.62500000000,	3278.35714285714,	2648.08928571429};
        double[] pixelsY_test = {2700.90178571429,	2526.79464285714,	2331.79464285714,	2478.04464285714};

        CameraPinholeRadial cameraDistortionCoefficients = CameraIntrinsics.exampleCameraPinholeRadial();

        PoseFrom3D2DPointMatches app = new LocalisePnP_BoofCV();

        int numPointsToUse = 1000000;
        if(args.length>0) {
            numPointsToUse = Integer.parseInt(args[0]);
        }
        System.out.println("Estimate camera pose from real image, measured pixels, and measured world points.");
        Se3_F64 refinedWorldToCamera = app.estimateCameraPoseFrom3D2DPointMatches(cameraDistortionCoefficients, numPointsToUse, worldX_test, worldY_test, pixelsX_test, pixelsY_test);

        System.out.println("refinedWorldToCamera:");
        int numElementsInRot = refinedWorldToCamera.R.getNumElements();
        Vector3D_F64    translation  = refinedWorldToCamera.T;
        System.out.println("rotation numElements="+numElementsInRot+", translation ="+translation.toString());
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
    @Override
    public Se3_F64 estimateCameraPoseFrom3D2DPointMatches(CameraPinholeRadial cameraDistortionCoefficients, int numPointsToUse, double[] worldX, double[] worldY, double[] pixelsX, double[] pixelsY) {
        return estimateCameraPoseFrom3D2DPointMatchesStatic(cameraDistortionCoefficients,  numPointsToUse,  worldX,  worldY,  pixelsX,  pixelsY);
    }

    public static Se3_F64 estimateCameraPoseFrom3D2DPointMatchesStatic(CameraPinholeRadial cameraDistortionCoefficients, int numPointsToUse, double[] worldX, double[] worldY, double[] pixelsX, double[] pixelsY) {
        // CameraPinholeRadial(double fx, double fy, double skew, double cx, double cy, int width, int height)

        List<Point2D3D> observations = new ArrayList<Point2D3D>();

        // transform from pixel coordinates to normalized pixel coordinates and removes lens distortion - note: rectified/undistorted AND normalised to range -1:1 - see http://answers.opencv.org/question/83807/normalized-camera-image-coordinates/
        Point2Transform2_F64 pixelToNormAndRectTrans = LensDistortionOps.transformPoint(cameraDistortionCoefficients).undistort_F64(true,false);
        Point2D_F64 normalisedAndRectPoint = new Point2D_F64();
        double x,y,z;
        z=0;
        double x_pixel,y_pixel;
        for (int i_ = 0; i_ < worldX.length && i_< numPointsToUse; i_++) {
            x=worldX[i_];          y=worldY[i_];               z=0;  // world 3d coordinates
            x_pixel=pixelsX[i_];   y_pixel=pixelsY[i_];
            pixelToNormAndRectTrans.compute(x_pixel,y_pixel,normalisedAndRectPoint); //rectify and normalise
            addToObservations( x, y, z, normalisedAndRectPoint,  observations ); // Save the observation
        }
        // get the PnP estimate
//        Se3_F64 refinedWorldToCamera = estimateNoOutliers(observations);
        Se3_F64 refinedWorldToCamera = estimateOutliers(cameraDistortionCoefficients, observations);
        return refinedWorldToCamera;
    }

    /** ****************************************
     */
    public static void addToObservations(double x, double y, double z, Point2D_F64 normalisedAndRectifiedImagePoint, List<Point2D3D> observations ) {
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
    public static Se3_F64 estimateNoOutliers( List<Point2D3D> observations ) {

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
    public static Se3_F64 estimateOutliers(CameraPinholeRadial cameraDistortionCoefficients, List<Point2D3D> observations ) {
        // We can no longer trust that each point is a real observation.  Let's use RANSAC to separate the points
        // You will need to tune the number of iterations and inlier threshold!!!
        Ransac<Se3_F64,Point2D3D> ransac =
                FactoryMultiViewRobust.pnpRansac(new ConfigPnP(cameraDistortionCoefficients),new ConfigRansac(300,1.0));

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

        /*  add each point : measured by hand, have errors of 0-10mm and 0-3 pixels in each dimension for each point.
            worldpoints =
                    1280        1920        1920        1280        1280        -640        -640        3200        3200         320        -960
                    1920        1920        2240        2240        3200        3200        3520        3520        4160        4160        4160
            pixels
            2839.60714285714	3518.62500000000	3278.35714285714	2648.08928571429	2240.67857142857	332.464285714285	377.732142857142	3473.35714285714	3184.33928571429	1265.67857142857
            2700.90178571429	2526.79464285714	2331.79464285714	2478.04464285714	2000.99107142857	2345.72321428571	2182.06250000000	1666.70535714286	1527.41964285714	1816.43750000000
        */
        /*  test #1 : full dataset.
            double[] worldX  = {1280,               1920,               1920,               1280,               1280,               -640,               -640,               3200,               3200,               320};
            double[] worldY  = {1920,               1920,               2240,               2240,               3200,               3200,               3520,               3520,               4160,               4160};
            double[] pixelsX = {2839.60714285714,	3518.62500000000,	3278.35714285714,	2648.08928571429,	2240.67857142857,	332.464285714285,	377.732142857142,	3473.35714285714,	3184.33928571429,
            1265.67857142857};
            double[] pixelsY = {2700.90178571429,	2526.79464285714,	2331.79464285714,	2478.04464285714,	2000.99107142857,	2345.72321428571,	2182.06250000000,	1666.70535714286,	1527.41964285714,
            1816.43750000000};
        */


}
