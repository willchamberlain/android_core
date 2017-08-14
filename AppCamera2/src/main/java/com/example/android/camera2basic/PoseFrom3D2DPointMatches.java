package com.example.android.camera2basic;

import boofcv.struct.calib.CameraPinholeRadial;
import georegression.struct.se.Se3_F64;

/**
 * Created by will on 3/08/17.
 */

interface PoseFrom3D2DPointMatches {
    Se3_F64 estimateCameraPoseFrom3D2DPointMatches(CameraPinholeRadial cameraDistortionCoefficients, int numPointsToUse, double[] worldX, double[] worldY, double[] worldZ, double[] pixelsX, double[] pixelsY);
    Se3_F64 estimateCameraPoseQuad(CameraPinholeRadial cameraDistortionCoefficients, double[] worldX, double[] worldY, double[] worldZ, double[] pixelsX, double[] pixelsY);
    }
