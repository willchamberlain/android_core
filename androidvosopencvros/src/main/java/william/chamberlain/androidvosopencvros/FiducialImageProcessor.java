package william.chamberlain.androidvosopencvros;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;

import william.chamberlain.androidvosopencvros.AlgorithmParameters;

import boofcv.abst.fiducial.CalibrationFiducialDetector;
import boofcv.abst.fiducial.FiducialDetector;
import boofcv.abst.fiducial.SquareBase_to_FiducialDetector;
import boofcv.abst.fiducial.calib.CalibrationDetectorChessboard;
import boofcv.abst.fiducial.calib.CalibrationDetectorSquareGrid;
import boofcv.abst.geo.calibration.DetectorFiducialCalibration;
import boofcv.alg.distort.LensDistortionOps;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.android.ConvertBitmap;
import boofcv.android.VisualizeImageData;
import boofcv.android.gui.VideoImageProcessing;
import boofcv.core.image.ConvertImage;
import boofcv.struct.calib.CameraPinholeRadial;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.ImageBase;
import boofcv.struct.image.ImageType;
import boofcv.struct.image.Planar;
import georegression.struct.point.Point2D_F32;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;

/**
 *
 * See /mnt/nixbig/downloads/boofcv_will/BoofAndroidDemo/app/src/main/java/org/boofcv/android/recognition/FiducialImageProcessor.java
 * See org.boofcv.android.recognition.FiducialImageProcessor
 * Created by will on 4/04/17.
 */
class FiducialImageProcessor<T extends ImageBase> extends VideoImageProcessing<Planar<GrayU8>> {

    T input;

    FiducialDetector<T> detector;

    Paint paintSelected = new Paint();
    Paint paintLine0 = new Paint();
    Paint paintLine1 = new Paint();
    Paint paintLine2 = new Paint();
    Paint paintLine3 = new Paint();
    private Paint textPaint = new Paint();
    private Paint textBorder = new Paint();

    Rect bounds = new Rect();

    boofcv.struct.calib.CameraModel cameraModel;
    HasCameraIntrinsics hasCameraIntrinsics;
    DetectorSource detectorSource;
    AlgorithmParameters algorithmParams;
    ApplicationPreferences preferences;

    protected FiducialImageProcessor(boofcv.struct.calib.CameraModel cameraModel_, HasCameraIntrinsics hasCameraIntrinsics_, DetectorSource detectorSource_, AlgorithmParameters algorithmParams_, ApplicationPreferences preferences_) {
        super(ImageType.pl(3, GrayU8.class));
        this.cameraModel            = cameraModel_;
        this.hasCameraIntrinsics    = hasCameraIntrinsics_;
        this.detectorSource         = detectorSource_;
        this.algorithmParams        = algorithmParams_;
        this.preferences            = preferences_;

        paintSelected.setColor(Color.argb(0xFF / 2, 0xFF, 0, 0));

        paintLine0.setColor(Color.RED);
        paintLine0.setStrokeWidth(4f);
        paintLine0.setFlags(Paint.ANTI_ALIAS_FLAG);
        paintLine1.setColor(Color.BLACK);
        paintLine1.setStrokeWidth(4f);
        paintLine1.setFlags(Paint.ANTI_ALIAS_FLAG);
        paintLine2.setColor(Color.BLUE);
        paintLine2.setStrokeWidth(4f);
        paintLine2.setFlags(Paint.ANTI_ALIAS_FLAG);
        paintLine3.setColor(Color.GREEN);
        paintLine3.setStrokeWidth(4f);
        paintLine3.setFlags(Paint.ANTI_ALIAS_FLAG);

        // Create out paint to use for drawing
        textPaint.setARGB(255, 255, 100, 100);
        textPaint.setTextSize(20);

        textBorder.setARGB(255, 0, 0, 0);
        textBorder.setTextSize(20);
        textBorder.setStyle(Paint.Style.STROKE);
        textBorder.setStrokeWidth(3);
    }

    @Override
    protected void declareImages(int width, int height) {
        super.declareImages(width, height);                     // VideoImageProcessing.declareImages --> declares the output images and 'storage' for them

        /*fiducialSquareActivity.cameraIntrinsics */
        cameraModel = CameraUtils.checkThenInventIntrinsic(hasCameraIntrinsics);
    }

    @Override
    protected void process(Planar<GrayU8> color, Bitmap output, byte[] storage) {
        if (algorithmParams.parametersHaveChanged() && hasCameraIntrinsics.intrinsics() != null) {   // (1) user has changed the slider control for thresholding  and  (2) cameraIntrinsics parameters are known
            algorithmParams.parametersHaveChanged(false);
            detector = (FiducialDetector) detectorSource.createDetector();     // FiducialSquareBinaryActivity.createDetector --> FiducialDetectionManager.createDetector
            detector.setLensDistortion(LensDistortionOps.transformPoint(hasCameraIntrinsics.intrinsics())); // CameraPinholeRadial model of instrinsic parameters
            if (input == null || input.getImageType() != detector.getInputType()) {         // input : <T extends ImageBase> --> input is one of GrayU8, GrayU16, etc
                input = detector.getInputType().createImage(1, 1);
            }
        }

        if (detector == null) {
            return;
        }

        ImageType inputType = detector.getInputType();
        if (inputType.getFamily() == ImageType.Family.GRAY) {
            input.reshape(color.width, color.height);
            ConvertImage.average(color, (GrayU8) input);                                    // averages the colour channels
        } else {
            input = (T) color;
        }

        detector.detect(input);

        if ( preferences.showInputImage) {
            ConvertBitmap.multiToBitmap(color, output, storage);
        } else {
            GrayU8 binary = null;
            if (detector instanceof CalibrationFiducialDetector) {
                DetectorFiducialCalibration a = ((CalibrationFiducialDetector) detector).getCalibDetector();
                if (a instanceof CalibrationDetectorChessboard) {
                    binary = ((CalibrationDetectorChessboard) a).getAlgorithm().getBinary();
                } else {
                    binary = ((CalibrationDetectorSquareGrid) a).getAlgorithm().getBinary();
                }
            } else {
                binary = ((SquareBase_to_FiducialDetector) detector).getAlgorithm().getBinary();    // here -
            }
            VisualizeImageData.binaryToBitmap(binary, false, output, storage);
        }

        Canvas canvas = new Canvas(output);
        Se3_F64 targetToCamera = new Se3_F64();

        for (int i = 0; i < detector.totalFound(); i++) {
            detector.getFiducialToCamera(i, targetToCamera);

            double width = detector.getWidth(i);
            drawCube(detector.getId(i), targetToCamera, hasCameraIntrinsics.intrinsics(), width, canvas);
        }

//        if (preferences.drawText) {
//            renderDrawText(canvas);
//        }

    }

    /**
     * Draws a flat cube to show where the square fiducial is on the image
     */
    public void drawCube(long number, Se3_F64 targetToCamera, CameraPinholeRadial intrinsic, double width,
                         Canvas canvas) {
        double r = width / 2.0;
        Point3D_F64 corners[] = new Point3D_F64[8];
        corners[0] = new Point3D_F64(-r, -r, 0);
        corners[1] = new Point3D_F64(r, -r, 0);
        corners[2] = new Point3D_F64(r, r, 0);
        corners[3] = new Point3D_F64(-r, r, 0);
        corners[4] = new Point3D_F64(-r, -r, r);
        corners[5] = new Point3D_F64(r, -r, r);
        corners[6] = new Point3D_F64(r, r, r);
        corners[7] = new Point3D_F64(-r, r, r);

        Point2D_F32 pixel[] = new Point2D_F32[8];
        Point2D_F64 p = new Point2D_F64();
        for (int i = 0; i < 8; i++) {
            Point3D_F64 c = corners[i];
            SePointOps_F64.transform(targetToCamera, c, c);
            PerspectiveOps.convertNormToPixel(intrinsic, c.x / c.z, c.y / c.z, p);
            pixel[i] = new Point2D_F32((float) p.x, (float) p.y);
        }

        Point3D_F64 centerPt = new Point3D_F64();

        SePointOps_F64.transform(targetToCamera, centerPt, centerPt);
        PerspectiveOps.convertNormToPixel(intrinsic,
                centerPt.x / centerPt.z, centerPt.y / centerPt.z, p);
        Point2D_F32 centerPixel = new Point2D_F32((float) p.x, (float) p.y);

        // red
        drawLine(canvas, pixel[0], pixel[1], paintLine0);
        drawLine(canvas, pixel[1], pixel[2], paintLine0);
        drawLine(canvas, pixel[2], pixel[3], paintLine0);
        drawLine(canvas, pixel[3], pixel[0], paintLine0);

        // black
        drawLine(canvas, pixel[0], pixel[4], paintLine1);
        drawLine(canvas, pixel[1], pixel[5], paintLine1);
        drawLine(canvas, pixel[2], pixel[6], paintLine1);
        drawLine(canvas, pixel[3], pixel[7], paintLine1);

        drawLine(canvas, pixel[4], pixel[5], paintLine2);
        drawLine(canvas, pixel[5], pixel[6], paintLine2);
        drawLine(canvas, pixel[6], pixel[7], paintLine2);
        drawLine(canvas, pixel[7], pixel[4], paintLine3);

        String numberString = "" + number;

        textPaint.getTextBounds(numberString, 0, numberString.length(), bounds);

        int textLength = bounds.width();
        int textHeight = bounds.height();

        canvas.drawText(numberString, centerPixel.x - textLength / 2, centerPixel.y + textHeight / 2, textBorder);
        canvas.drawText(numberString, centerPixel.x - textLength / 2, centerPixel.y + textHeight / 2, textPaint);


        Se3_F64 cameraToTarget = null;
        cameraToTarget = targetToCamera.invert(cameraToTarget);
        textPaint.setARGB(255, 255, 100, 100);
        textPaint.setTextSize(20);
        String string = "x=" + String.valueOf(cameraToTarget.getX()).substring(0, 5);
        canvas.drawText(string, centerPixel.x - textLength / 2, centerPixel.y + textHeight + 10, textPaint);
        string = "y=" + String.valueOf(cameraToTarget.getY()).substring(0, 5);
        canvas.drawText(string, centerPixel.x - textLength / 2, centerPixel.y + textHeight * 2 + 10, textPaint);
        string = "z=" + String.valueOf(cameraToTarget.getZ()).substring(0, 5);
        canvas.drawText(string, centerPixel.x - textLength / 2, centerPixel.y + textHeight * 3 + 10, textPaint);

        textPaint.setARGB(255, 100, 100, 255);
        textPaint.setTextSize(25);
        string = "x=" + String.valueOf(cameraToTarget.getZ()).substring(0, 5);
        canvas.drawText(string, centerPixel.x - textLength / 2, centerPixel.y + textHeight * 4 + 10, textPaint);
        string = "y=" + String.valueOf(0 - cameraToTarget.getX()).substring(0, 5);
        canvas.drawText(string, centerPixel.x - textLength / 2, centerPixel.y + textHeight * 5 + 10, textPaint);
        string = "z=" + String.valueOf(0 - cameraToTarget.getY()).substring(0, 5);
        canvas.drawText(string, centerPixel.x - textLength / 2, centerPixel.y + textHeight * 6 + 10, textPaint);
    }

//    private void renderDrawText(Canvas canvas) {
//        textPaint.getTextBounds(fiducialSquareActivity.drawText, 0, fiducialSquareActivity.drawText.length(), bounds);
//
//        int textLength = bounds.width();
//        int textHeight = bounds.height();
//
//        int x0 = canvas.getWidth() / 2 - textLength / 2;
//        int y0 = canvas.getHeight() / 2 + textHeight / 2;
//
//        canvas.drawText(fiducialSquareActivity.drawText, x0, y0, textBorder);
//        canvas.drawText(fiducialSquareActivity.drawText, x0, y0, textPaint);
//    }

    private void drawLine(Canvas canvas, Point2D_F32 a, Point2D_F32 b, Paint color) {
        canvas.drawLine(a.x, a.y, b.x, b.y, color);
    }

}
