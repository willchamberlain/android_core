/*
 * Copyright 2014 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.example.android.camera2basic;

import android.Manifest;
import android.app.Activity;
import android.app.AlertDialog;
import android.app.Dialog;
import android.app.DialogFragment;
import android.app.Fragment;
import android.content.Context;
import android.content.DialogInterface;
import android.content.pm.PackageManager;
import android.content.res.Configuration;
import android.graphics.ImageFormat;
import android.graphics.Matrix;
import android.graphics.Point;
import android.graphics.RectF;
import android.graphics.SurfaceTexture;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.TotalCaptureResult;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Process;
import android.support.annotation.NonNull;
import android.support.v13.app.FragmentCompat;
import android.support.v4.content.ContextCompat;
import android.util.Log;
import android.util.Range;
import android.util.Size;
import android.util.SparseIntArray;
import android.view.LayoutInflater;
import android.view.Surface;
import android.view.TextureView;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Toast;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Random;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.Semaphore;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

import boofcv.abst.fiducial.FiducialDetector;
import boofcv.alg.distort.LensDistortionNarrowFOV;
import boofcv.alg.distort.pinhole.LensDistortionPinhole;
import boofcv.alg.distort.radtan.LensDistortionRadialTangential;
import boofcv.core.encoding.ConvertNV21;
import boofcv.factory.fiducial.ConfigFiducialBinary;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.factory.filter.binary.ConfigThreshold;
import boofcv.factory.filter.binary.ThresholdType;
import boofcv.struct.calib.CameraPinhole;
import boofcv.struct.calib.CameraPinholeRadial;
import boofcv.struct.image.GrayU8;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.EulerType;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Quaternion_F64;
import william.chamberlain.androidvosopencvros.DetectedTag;
import william.chamberlain.androidvosopencvros.Hardcoding;
import william.chamberlain.androidvosopencvros.LandmarkFeatureLoader;
import william.chamberlain.androidvosopencvros.RobotId;
import william.chamberlain.androidvosopencvros.RosThingy;
import william.chamberlain.androidvosopencvros.VosTaskSet;

import static william.chamberlain.androidvosopencvros.Hardcoding.MARKER_OFFSET_INT;


public class Camera2BasicFragment extends Fragment
        implements View.OnClickListener, FragmentCompat.OnRequestPermissionsResultCallback {

    /**
     * Conversion from screen rotation to JPEG orientation.
     */
    private static final SparseIntArray ORIENTATIONS = new SparseIntArray();
    private static final int REQUEST_CAMERA_PERMISSION = 1;
    private static final String FRAGMENT_DIALOG = "dialog";
    static final int targetFPS = 3; //4; //8;
    static final int numRecordsToUse = 10;
    public static final float FX_FOCAL_LENGTH_X_PIXELS_AS_CALIBRATED = 528.65F;  // Nexus 6 # 510  //= 263.1f*2.0f; // 519.902859f;
    public static final float FY_FOCAL_LENGTH_Y_PIXELS_AS_CALIBRATED = 527.93F;  // Nexus 6 # 510  //= 262.8f*2.0f; // 518.952669f;
    public static final float CX_FOCAL_MIDPOINT_X_PIXELS_AS_CALIBRATED = 325.23F;  // Nexus 6 # 510
    public static final float CY_FOCAL_MIDPOINT_Y_PIXELS_AS_CALIBRATED = 236.43F;  // Nexus 6 # 510
    public static final float CX = 325.23F;  // Nexus 6 # 510
    public static final float CY = 236.43F;  // Nexus 6 # 510
    public static final double SKEW_PIXELS_AS_CALIBRATED = 0.0;
    public static final float RADIAL_1_AS_CALIBRATED = 7.22F*0.01F;
    public static final float RADIAL_2_AS_CALIBRATED = -1.49F*0.1F;
    public static final float IMAGE_WIDTH_PIXELS_AS_CALIBRATED = 640.0f;
    public static final float IMAGE_HEIGHT_PIXELS_AS_CALIBRATED = 480.0f;
    static int fps = 3; //5;
    public static  Range<Integer> fpsRange = new Range<>(fps, fps);

    static {
        ORIENTATIONS.append(Surface.ROTATION_0, 90);
        ORIENTATIONS.append(Surface.ROTATION_90, 0);
        ORIENTATIONS.append(Surface.ROTATION_180, 270);
        ORIENTATIONS.append(Surface.ROTATION_270, 180);
    }

    /**
     * Tag for the {@link Log}.
     */
    private static final String TAG = "Camera2BasicFragment";

    /**
     * Camera state: Showing camera preview.
     */
    private static final int STATE_SHOWING_CAMERA_PREVIEW = 0;

    /**
     * Camera state: Waiting for the focus to be locked.
     */
    private static final int STATE_WAITING_FOR_FOCUS_LOCK = 1;

    /**
     * Camera state: Waiting for the exposure to be precapture state.
     */
    private static final int STATE_WAITING_FOR_EXPOSURE_TO_BE_PRECAPTURE = 2;

    /**
     * Camera state: Waiting for the exposure state to be something other than precapture.
     */
    private static final int STATE_WAITING_FOR_EXPOSURE_TO_NOT_BE_PRECAPTURE = 3;

    /**
     * Camera state: Picture was taken.
     */
    private static final int STATE_PICTURE_TAKEN = 4;

    /**
     * Max preview width that is guaranteed by Camera2 API
     */
    private static final int MAX_PREVIEW_WIDTH = 640; //1920;

    /**
     * Max preview height that is guaranteed by Camera2 API
     */
    private static final int MAX_PREVIEW_HEIGHT = 480; //1080;

    /**
     * {@link TextureView.SurfaceTextureListener} handles several lifecycle events on a
     * {@link TextureView}.
     */
    private final TextureView.SurfaceTextureListener mSurfaceTextureListener
            = new TextureView.SurfaceTextureListener() {

        @Override
        public void onSurfaceTextureAvailable(SurfaceTexture texture, int width, int height) {
            openCamera(width, height);
        }

        @Override
        public void onSurfaceTextureSizeChanged(SurfaceTexture texture, int width, int height) {
            configureTransform(width, height);
        }

        @Override
        public boolean onSurfaceTextureDestroyed(SurfaceTexture texture) {
            return true;
        }

        @Override
        public void onSurfaceTextureUpdated(SurfaceTexture texture) {
        }

    };

    /**
     * ID of the current {@link CameraDevice}.
     */
    private String cameraId;

    /**
     * An {@link AutoFitTextureView} for camera preview.
     */
    private AutoFitTextureView mTextureView;

    /**
     * A {@link CameraCaptureSession } for camera preview.
     */
    private CameraCaptureSession mCaptureSession;

    /**
     * A reference to the opened {@link CameraDevice}.
     */
    private CameraDevice mCameraDevice;

    /**
     * The {@link android.util.Size} of camera preview.
     */
    private Size imageSize;


    /**
     * {@link CameraDevice.StateCallback} is called when {@link CameraDevice} changes its state.
     */
    private final CameraDevice.StateCallback mStateCallback = new CameraDevice.StateCallback() {

        @Override
        public void onOpened(@NonNull CameraDevice cameraDevice) {
            // This method is called when the camera is opened.  We start camera preview here.
            mCameraOpenCloseLock.release();
            mCameraDevice = cameraDevice;
            createCameraPreviewSession();
        }

        @Override
        public void onDisconnected(@NonNull CameraDevice cameraDevice) {
            mCameraOpenCloseLock.release();
            cameraDevice.close();
            mCameraDevice = null;
        }

        @Override
        public void onError(@NonNull CameraDevice cameraDevice, int error) {
            mCameraOpenCloseLock.release();
            cameraDevice.close();
            mCameraDevice = null;
            Activity activity = getActivity();
            if (null != activity) {
                activity.finish();
            }
        }

    };

    /**
     * An additional thread for running tasks that shouldn't block the UI.
     */
    private HandlerThread mBackgroundThread;
    private HandlerThread mBackgroundThread2;

    /**
     * A {@link Handler} for running tasks in the background.
     */
    private Handler backgroundHandler;
    private Handler mBackgroundHandler2;

    /**
     * An {@link ImageReader} that handles still image capture.
     */
    private ImageReader mImageReader;

    /**
     * This is the output file for our picture.
     */
    private File mFile;

    private int skipRate = 10; // initialise to be over 10
    long frameNum = 0;
    long skipRateReducedOnFrameNum = 0;
    TaskCompletionTimer taskCompletionTimer = TaskCompletionTimer.instance();


    /**********************************************************************************************/

    final int RUN_SYNCHRONOUS =  0;
    final int RUN_ASYNC_BACKGROUNDHANDLER =  10;
    final int RUN_ASYNC_ASYNCTASK =         20;
    final int RUN_ASYNC_METHOD =  RUN_ASYNC_ASYNCTASK; // RUN_ASYNC_BACKGROUNDHANDLER; //RUN_ASYNC_ASYNCTASK; // RUN_SYNCHRONOUS;  // RUN_ASYNC_ASYNCTASK;

    TaskHolder taskHolder = null;
    boolean currentlyProcessing = false;

    /**
     * Do the image processing - this a callback object for the {@link ImageReader}. "onImageAvailable" will be called when a
     * still image is ready to be saved.
     */
    private final ImageReader.OnImageAvailableListener onImageAvailableListener
            = new ImageReader.OnImageAvailableListener() {

        @Override
        public void onImageAvailable(ImageReader reader) {
            frameNum++;  if (frameNum == Long.MAX_VALUE) { frameNum=1; }

            long registerAsVisionSourceStartTime = Calendar.getInstance().getTimeInMillis();
            rosThingy.registerAsVisionSource();
            Log.i("Camera2BasicFragment", "onImageAvailable: ROS timing: rosThingy.updateLocationFromDetectedFeature took "+timeElapsed(registerAsVisionSourceStartTime)+"ms");
            Log.i("Camera2BasicFragment", "onImageAvailable: ROS timing: rosThingy.updateLocationFromDetectedFeature + LOGGING took "+timeElapsed(registerAsVisionSourceStartTime)+"ms");

            switch (RUN_ASYNC_METHOD) {
                case RUN_SYNCHRONOUS: {
                    Image image = null;
                    try {
                        if (currentlyProcessing) {
                            Log.i("Camera2BasicFragment", "onImageAvailable: currentlyProcessing so returning");
                            return;
                        }
                        if (null == taskHolder) {
                            Log.i("Camera2BasicFragment", "onImageAvailable: taskHolder = new TaskHolder()");
                            taskHolder = new TaskHolder();
                        }
                        currentlyProcessing = true;
                        image = reader.acquireNextImage();       // TODO see https://stackoverflow.com/a/43564630/1200764
                            // reader.getMaxImages();
                        taskHolder.setup(image, TaskCompletionTimer.instance(), Calendar.getInstance().getTimeInMillis());
                        taskHolder.imageProcessingAndRosCalling();
                        currentlyProcessing = false;
                    } finally {
                        if (image != null) {             // ?? causes an exception because !when the app starts! closed before can save                // TODO see https://stackoverflow.com/a/43564630/1200764
                            image.close();               // ?? causes an exception because !when the app starts! closed before can save                // TODO see https://stackoverflow.com/a/43564630/1200764
                        }
                    }
                    break;
                }
                case RUN_ASYNC_BACKGROUNDHANDLER: {
                    Image image = null;
                    try {
                        image = reader.acquireNextImage();       // TODO see https://stackoverflow.com/a/43564630/1200764
                    } catch (IllegalStateException e) {
                        break;
                    }
                    if (null != backgroundHandler) {
                        backgroundHandler.post(
                                new ImageSaver(
                                        image /*,                          // TODO see https://stackoverflow.com/a/43564630/1200764
                            mFile */));  // push ImageSaver runnable/task onto the backgroundthread's message queue to be executed on the backgroundthread
                        if (image != null) {             // ?? causes an exception because !when the app starts! closed before can save                // TODO see https://stackoverflow.com/a/43564630/1200764
                            image.close();               // ?? causes an exception because !when the app starts! closed before can save                // TODO see https://stackoverflow.com/a/43564630/1200764
                        }
                    }
                    break;
                }
                case RUN_ASYNC_ASYNCTASK: {Image image = null;
                    try {
                        image = reader.acquireNextImage();       // TODO see https://stackoverflow.com/a/43564630/1200764
                    } catch (IllegalStateException e) {
                        break;
                    }
                    if (skipFrame(image)) {
                        if (image != null) {             // ?? causes an exception because !when the app starts! closed before can save                // TODO see https://stackoverflow.com/a/43564630/1200764
                            image.close();               // ?? causes an exception because !when the app starts! closed before can save                // TODO see https://stackoverflow.com/a/43564630/1200764
                        }
                        break;
                    }
                    try {
                        //  new ImageSaverAsyncTask(image, taskCompletionTimer).executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR);  // https://stackoverflow.com/questions/18266161/achieving-concurrency-using-asynctask  https://developer.android.com/reference/java/util/concurrent/Executor.html
                        new ImageSaverAsyncTask(image, taskCompletionTimer).executeOnExecutor(threadPoolExecutor);  // https://stackoverflow.com/questions/18266161/achieving-concurrency-using-asynctask  https://developer.android.com/reference/java/util/concurrent/Executor.html
                        if (image != null) {             // ?? causes an exception because !when the app starts! closed before can save                // TODO see https://stackoverflow.com/a/43564630/1200764
                            image.close();               // ?? causes an exception because !when the app starts! closed before can save                // TODO see https://stackoverflow.com/a/43564630/1200764
                        }
                    } catch (java.util.concurrent.RejectedExecutionException ree) {
                        Log.w("Camera2BasicFragment","onImageAvailable(ImageReader reader): hit the limit of available threads with a java.util.concurrent.RejectedExecutionException");
                        skipRateOnException();
                        if (image != null) {             // ?? causes an exception because !when the app starts! closed before can save                // TODO see https://stackoverflow.com/a/43564630/1200764
                            image.close();               // ?? causes an exception because !when the app starts! closed before can save                // TODO see https://stackoverflow.com/a/43564630/1200764
                        }
                    }
                    break;
                }
                default:
            }
        }
    };

    /**********************************************************************************************/

    private void skipRateOnException() {
        if(skipRate <= 10){ skipRate--; skipRateReducedOnFrameNum = frameNum; }
        if(skipRate > 10){ skipRate = 10; }
        if(skipRate < 2){ skipRate = 2; }
    }

    private boolean skipFrame(Image image) {
        if (skipRate <= 10 && skipRate>0 && frameNum%skipRate == 0 ) { // skip this image - e.g. if skipRate == 8 and frameNum == 16
            Log.i("Camera2BasicFragment","!! SKIPPING DISABLED !! skipping frame "+frameNum+" on skipRate "+skipRate);
            return true;
//            Log.i("Camera2BasicFragment","skipping frame "+frameNum+" on skipRate "+skipRate);
//            if (image != null) {             // ?? causes an exception because !when the app starts! closed before can save                // TODO see https://stackoverflow.com/a/43564630/1200764
//                image.close();               // ?? causes an exception because !when the app starts! closed before can save                // TODO see https://stackoverflow.com/a/43564630/1200764
//            }
//            return true;
        }
        int TARGET_FRAME_RATE = 10;
        if ( skipRateReducedOnFrameNum < frameNum - 100 && skipRate > 2 && taskCompletionTimer.overlapWithLastInitiation() > (5000/TARGET_FRAME_RATE)) { // more than 1s overlap; going wrong
            skipRate--; if(skipRate < 2){ skipRate = 2; }
            skipRateReducedOnFrameNum = frameNum;
            Log.i("Camera2BasicFragment","at frame "+frameNum+" reduced skipRate to "+skipRate+" on taskCompletionTimer.overlapWithLastInitiation() = "+taskCompletionTimer.overlapWithLastInitiation()+" > (5000/"+TARGET_FRAME_RATE+")");
        }
        return false;
    }

    /**
     * {@link CaptureRequest.Builder} for the camera preview
     */
    private CaptureRequest.Builder mPreviewRequestBuilder;

    /**
     * {@link CaptureRequest} generated by {@link #mPreviewRequestBuilder}
     */
    private CaptureRequest mPreviewRequest;

    /**
     * The current state of camera state for taking pictures.
     *
     * @see #mCaptureCallback
     */
    private int mState = STATE_SHOWING_CAMERA_PREVIEW;

    /**
     * A {@link Semaphore} to prevent the app from exiting before closing the camera.
     */
    private Semaphore mCameraOpenCloseLock = new Semaphore(1);

    /**
     * Whether the current camera device supports Flash or not.
     */
    private boolean mFlashSupported;

    /**
     * Orientation of the camera sensor
     */
    private int mSensorOrientation;


    /**********************************************************************************************/
    /**
     * A {@link CameraCaptureSession.CaptureCallback} that handles events related to JPEG capture.
     */
    private CameraCaptureSession.CaptureCallback mCaptureCallback
            = new CameraCaptureSession.CaptureCallback() {

        private void process(CaptureResult result) {
            switch (mState) {
                case STATE_SHOWING_CAMERA_PREVIEW: {
                    // We have nothing to do when the camera preview is working normally.
                    break;
                }
                case STATE_WAITING_FOR_FOCUS_LOCK: {
                    Integer afState = result.get(CaptureResult.CONTROL_AF_STATE); // check the auto-focus state
                    if (afState == null) {                                        // no particular auto-focus state: capture a frame
                        captureStillPicture();
                                                                                    // ? missing  mState = STATE_PICTURE_TAKEN;  ?
                    } else if (CaptureResult.CONTROL_AF_STATE_FOCUSED_LOCKED == afState ||      // autofocus is locked, may be able to capture a frame
                            CaptureResult.CONTROL_AF_STATE_NOT_FOCUSED_LOCKED == afState) {
                        // CONTROL_AE_STATE can be null on some devices
                        Integer aeState = result.get(CaptureResult.CONTROL_AE_STATE); // check the auto-exposure state
                        if (aeState == null ||                                        // no particular auto-exposure state or auto-exposure state is converged/ready: capture a frame
                                aeState == CaptureResult.CONTROL_AE_STATE_CONVERGED) {
                            mState = STATE_PICTURE_TAKEN;
                            captureStillPicture();
                        } else {
                            runPrecaptureSequenceAndCaptureImage();
                        }
                    }
                    break;
                }
                case STATE_WAITING_FOR_EXPOSURE_TO_BE_PRECAPTURE: {
                    // CONTROL_AE_STATE can be null on some devices
                    Integer aeState = result.get(CaptureResult.CONTROL_AE_STATE); // check the auto-exposure state
                    if (aeState == null ||
                            aeState == CaptureResult.CONTROL_AE_STATE_PRECAPTURE ||
                            aeState == CaptureRequest.CONTROL_AE_STATE_FLASH_REQUIRED) {
                        mState = STATE_WAITING_FOR_EXPOSURE_TO_NOT_BE_PRECAPTURE;
                    }
                    break;
                }
                case STATE_WAITING_FOR_EXPOSURE_TO_NOT_BE_PRECAPTURE: {
                    // CONTROL_AE_STATE can be null on some devices
                    Integer aeState = result.get(CaptureResult.CONTROL_AE_STATE); // check the auto-exposure state
                    if (aeState == null ||
                            aeState != CaptureResult.CONTROL_AE_STATE_PRECAPTURE) {
                        mState = STATE_PICTURE_TAKEN;
                        captureStillPicture();
                    }
                    break;
                }
            }
        }

        @Override
        public void onCaptureProgressed(@NonNull CameraCaptureSession session,
                                        @NonNull CaptureRequest request,
                                        @NonNull CaptureResult partialResult) {
            process(partialResult);
        }

        @Override
        public void onCaptureCompleted(@NonNull CameraCaptureSession session,
                                       @NonNull CaptureRequest request,
                                       @NonNull TotalCaptureResult result) {
            process(result);
        }

    };
    /**********************************************************************************************/

    /**
     * Shows a {@link Toast} on the UI thread.
     *
     * @param text The message to show
     */
    private void showToast(final String text) {
        final Activity activity = getActivity();
        if (activity != null) {
            activity.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    Toast.makeText(activity, text, Toast.LENGTH_SHORT).show();
                }
            });
        }
    }

    /**
     * Given {@code choices} of {@code Size}s supported by a camera, choose the smallest one that
     * is at least as large as the respective texture view size, and that is at most as large as the
     * respective max size, and whose aspect ratio matches with the specified value. If such size
     * doesn't exist, choose the largest one that is at most as large as the respective max size,
     * and whose aspect ratio matches with the specified value.
     *
     * @param choices           The list of sizes that the camera supports for the intended output
     *                          class
     * @param textureViewWidth  The width of the texture view relative to sensor coordinate
     * @param textureViewHeight The height of the texture view relative to sensor coordinate
     * @param maxWidth          The maximum width that can be chosen
     * @param maxHeight         The maximum height that can be chosen
     * @param aspectRatio       The aspect ratio
     * @return The optimal {@code Size}, or an arbitrary one if none were big enough
     */
    private static Size chooseOptimalSizeForImage(Size[] choices, int textureViewWidth,
                                                  int textureViewHeight, int maxWidth, int maxHeight, Size aspectRatio) {

        // Collect the supported resolutions that are at least as big as the preview Surface
        List<Size> bigEnough = new ArrayList<>();
        // Collect the supported resolutions that are smaller than the preview Surface
        List<Size> notBigEnough = new ArrayList<>();
        int w = aspectRatio.getWidth();
        int h = aspectRatio.getHeight();
        for (Size option : choices) {
            if (option.getWidth() <= maxWidth && option.getHeight() <= maxHeight &&
                    option.getHeight() == option.getWidth() * h / w) {
                if (option.getWidth() >= textureViewWidth &&
                    option.getHeight() >= textureViewHeight) {
                    bigEnough.add(option);
                } else {
                    notBigEnough.add(option);
                }
            }
        }

        // Pick the smallest of those big enough. If there is no one big enough, pick the
        // largest of those not big enough.
        if (bigEnough.size() > 0) {
            return Collections.min(bigEnough, new CompareSizesByArea());
        } else if (notBigEnough.size() > 0) {
            return Collections.max(notBigEnough, new CompareSizesByArea());
        } else {
            Log.e(TAG, "Couldn't find any suitable preview size");
            return choices[0];
        }
    }

    public static Camera2BasicFragment newInstance() {
        Log.i("Camera2BasicFragment","newInstance(): start");
        return new Camera2BasicFragment();
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        return inflater.inflate(R.layout.fragment_camera2_basic, container, false);
    }

    @Override
    public void onViewCreated(final View view, Bundle savedInstanceState) {
        //    hookUpGuiButtons(view);
        mTextureView = (AutoFitTextureView) view.findViewById(R.id.texture);
    }

        //    private void hookUpGuiButtons(View view) {
        //        view.findViewById(R.id.picture).setOnClickListener(this);
        //        view.findViewById(R.id.fps).setOnClickListener(this);
        //        view.findViewById(R.id.info).setOnClickListener(this);
        //    }

    @Override
    public void onActivityCreated(Bundle savedInstanceState) {
        super.onActivityCreated(savedInstanceState);
        mFile = new File(getActivity().getExternalFilesDir(null), "pic.jpg");
    }


    VosTaskSet vosTaskSet;

    @Override
    public void onResume() {
        super.onResume();

        Log.i("Camera2BasicFragment","onResume(): start");


        startBackgroundThread();
        threadPoolExecutor.allowCoreThreadTimeOut(true);
        vosTaskSet = new VosTaskSet();
        Hardcoding.hardcodeTargetMarkers(vosTaskSet);


        // When the screen is turned off and turned back on, the SurfaceTexture is already
        // available, and "onSurfaceTextureAvailable" will not be called. In that case, we can open
        // a camera and start preview from here (otherwise, we wait until the surface is ready in
        // the SurfaceTextureListener).
        if (mTextureView.isAvailable()) {
            openCamera(mTextureView.getWidth(), mTextureView.getHeight());
        } else {
            mTextureView.setSurfaceTextureListener(mSurfaceTextureListener);
        }


    }

    @Override
    public void onPause() {
        Log.i("Camera2BasicFragment","onPause(): start");
        closeCamera();
        stopBackgroundThread();
        super.onPause();
    }

    private void requestCameraPermission() {
        if (FragmentCompat.shouldShowRequestPermissionRationale(this, Manifest.permission.CAMERA)) {
            new ConfirmationDialog().show(getChildFragmentManager(), FRAGMENT_DIALOG);
        } else {
            FragmentCompat.requestPermissions(this, new String[]{Manifest.permission.CAMERA},
                    REQUEST_CAMERA_PERMISSION);
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        if (requestCode == REQUEST_CAMERA_PERMISSION) {
            if (grantResults.length != 1 || grantResults[0] != PackageManager.PERMISSION_GRANTED) {
                ErrorDialog.newInstance(getString(R.string.request_permission))
                        .show(getChildFragmentManager(), FRAGMENT_DIALOG);
            }
        } else {
            super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        }
    }

    /**
     * Sets up member variables related to camera.
     *
     * @param width  The width of available size for camera preview
     * @param height The height of available size for camera preview
     */
    private void setUpCameraOutputs(int width, int height) {
        Activity activity = getActivity();
        CameraManager manager = (CameraManager) activity.getSystemService(Context.CAMERA_SERVICE);
        try {
            logCameraCharacteristics(manager);
            for (String cameraId_ : manager.getCameraIdList()) {
                CameraCharacteristics cameraDetails = manager.getCameraCharacteristics(cameraId_);

                if(isFrontFacingCameraId(cameraDetails)) { continue; } // We don't use a front facing camera in this sample.

                StreamConfigurationMap map = cameraDetails.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
                if (map == null) { continue; }

                // For still image captures, we use the largest available size.
                Size largest = Collections.max(
                        Arrays.asList(map.getOutputSizes(ImageFormat.YUV_420_888 /*ImageFormat.JPEG*/ )),         // TODO see https://stackoverflow.com/a/43564630/1200764
                        new CompareSizesByArea());



                // Find out if we need to swap dimension to get the preview size relative to sensor  coordinate.
                int displayRotation = activity.getWindowManager().getDefaultDisplay().getRotation();
                //noinspection ConstantConditions
                mSensorOrientation = cameraDetails.get(CameraCharacteristics.SENSOR_ORIENTATION);

                Point displaySize = new Point();
                activity.getWindowManager().getDefaultDisplay().getSize(displaySize);
                int rotatedPreviewWidth = width;         int rotatedPreviewHeight = height;
                int maxPreviewWidth     = displaySize.x; int maxPreviewHeight     = displaySize.y;
                boolean swappedDimensions = false;
                switch (displayRotation) {
                    case Surface.ROTATION_0: case Surface.ROTATION_180:
                        if (mSensorOrientation == 90 || mSensorOrientation == 270) { swappedDimensions = true; }
                        break;
                    case Surface.ROTATION_90: case Surface.ROTATION_270:
                        if (mSensorOrientation == 0 || mSensorOrientation == 180) { swappedDimensions = true; }
                        break;
                    default:
                        Log.e(TAG, "Display rotation is invalid: " + displayRotation);
                }
                if (swappedDimensions) {
                    rotatedPreviewWidth = height;         rotatedPreviewHeight = width;
                    maxPreviewWidth     = displaySize.y;  maxPreviewHeight     = displaySize.x;
                }
                if (maxPreviewWidth > MAX_PREVIEW_WIDTH) { maxPreviewWidth = MAX_PREVIEW_WIDTH; }
                if (maxPreviewHeight > MAX_PREVIEW_HEIGHT) { maxPreviewHeight = MAX_PREVIEW_HEIGHT; }

                // Danger, W.R.! Attempting to use too large a preview size could  exceed the camera
                // bus' bandwidth limitation, resulting in gorgeous previews but the storage of
                // garbage capture data.
                imageSize = chooseOptimalSizeForImage(map.getOutputSizes(SurfaceTexture.class),  // TODO - match the mImageReader = ImageReader.newInstance() to this preview size
                        rotatedPreviewWidth, rotatedPreviewHeight, 
                        maxPreviewWidth, maxPreviewHeight, largest);
                    Log.i(TAG, "setUpCameraOutputs(int "+width+", int "+height+"): mPreviewSize: "+imageSize.getWidth()+"x"+imageSize.getHeight());
                    /* TODO - "the ImageReader class allows direct application access to image data rendered into a {@link android.view.Surface"*/
                if (getResources().getConfiguration().orientation == Configuration.ORIENTATION_LANDSCAPE) {
                    mTextureView.setAspectRatio(imageSize.getWidth(), imageSize.getHeight());           // Fit the aspect ratio of TextureView to the size of preview.
                    mImageReader = ImageReader.newInstance( imageSize.getWidth(), imageSize.getHeight(), ImageFormat.YUV_420_888, 2 /*maxImages*/ ); // TODO see https://stackoverflow.com/a/43564630/1200764 - this scaling is arbitrary; reduces the max width from 4160 to 240, so always get 320x240
                } else {
                    mTextureView.setAspectRatio(imageSize.getHeight(), imageSize.getWidth());
                    mImageReader = ImageReader.newInstance( imageSize.getHeight(), imageSize.getWidth(), ImageFormat.YUV_420_888, 2 /*maxImages*/ ); // TODO see https://stackoverflow.com/a/43564630/1200764 - this scaling is arbitrary; reduces the max width from 4160 to 240, so always get 320x240
                }

                /* TODO - this configures the image processing as a callback that is called whenever a frame is available
                    - onImageAvailableListener is a  ImageReader.OnImageAvailableListener, which is called whenever mImageReader has a frame available to process
                    - backgroundHandler is a Handler
                 */
                mImageReader.setOnImageAvailableListener(onImageAvailableListener, backgroundHandler);
                boolean flashSupported = isFlashSupported(cameraDetails);
                cameraId = cameraId_;
                return;
            }
        } catch (CameraAccessException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            // Currently an NPE is thrown when the Camera2API is used but not supported on the
            // device this code runs.
            ErrorDialog.newInstance(getString(R.string.camera_error))
                    .show(getChildFragmentManager(), FRAGMENT_DIALOG);
        }
    }

    private boolean isFlashSupported(CameraCharacteristics cameraDetails) {
        // Check if the flash is supported.
        Boolean available = cameraDetails.get(CameraCharacteristics.FLASH_INFO_AVAILABLE);
        return available == null ? false : available;
    }

    private boolean isFrontFacingCameraId(CameraCharacteristics characteristics) {
        Integer facing = characteristics.get(CameraCharacteristics.LENS_FACING);
        boolean frontFacingCameraId = false;
        if (facing != null && facing == CameraCharacteristics.LENS_FACING_FRONT) {
            frontFacingCameraId = true;
        }
        return frontFacingCameraId;
    }

    /**
     * Opens the camera specified by {@link Camera2BasicFragment#cameraId}.
     */
    private void openCamera(int width, int height) {
        if (ContextCompat.checkSelfPermission(getActivity(), Manifest.permission.CAMERA)
                != PackageManager.PERMISSION_GRANTED) {
            requestCameraPermission();
            return;
        }
        setUpCameraOutputs(width, height);
        configureTransform(width, height);
        Activity activity = getActivity();
        CameraManager manager = (CameraManager) activity.getSystemService(Context.CAMERA_SERVICE);
        try {
            if (!mCameraOpenCloseLock.tryAcquire(2500, TimeUnit.MILLISECONDS)) {
                throw new RuntimeException("Time out waiting to lock camera opening.");
            }
            manager.openCamera(cameraId, mStateCallback, backgroundHandler);
            logCameraCharacteristics(manager);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        } catch (InterruptedException e) {
            throw new RuntimeException("Interrupted while trying to lock camera opening.", e);
        }
    }

    private void logCameraCharacteristics(CameraManager manager) throws CameraAccessException {
            Log.i(TAG,"logCameraCharacteristics: start");
        String[] cameras = manager.getCameraIdList();
        for(String camera : cameras) {
            CameraCharacteristics cc = manager.getCameraCharacteristics(camera);
            CameraCharacteristics.Key<int[]> aa = cc.REQUEST_AVAILABLE_CAPABILITIES;
            for (int i = 0; i < cc.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES).length; i++) {
                Log.i(TAG, "logCameraCharacteristics: camera="+camera+" available capability id= " + cc.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES)[i]);
            }
        }
        for(String camera : cameras) {
            CameraCharacteristics cc = manager.getCameraCharacteristics(camera);
            Range<Integer>[] fpsRange = cc.get(cc.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
                Log.i(TAG, "logCameraCharacteristics: camera="+camera+" CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES fpsRange= [" + fpsRange[0] + "," + fpsRange[1] + "]");
            StreamConfigurationMap map = manager.getCameraCharacteristics(camera).get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
            if(null != map) {
                fpsRange = map.getHighSpeedVideoFpsRanges(); // this range intends available fps range of device's camera.
                Log.i(TAG, "logCameraCharacteristics: camera=" + camera + " StreamConfigurationMap getHighSpeedVideoFpsRanges fpsRange=" + fpsRange.toString());
                try {   Log.i(TAG, "logCameraCharacteristics: camera="+camera+" StreamConfigurationMap getHighSpeedVideoFpsRanges fpsRange= [" + fpsRange[0] + "," + fpsRange[1] + "]"); }
                catch(Exception e) {Log.i(TAG, "logCameraCharacteristics: camera="+camera+" StreamConfigurationMap getHighSpeedVideoFpsRanges : exception getting fpsRange="+e);}
            }
        }
            Log.i(TAG,"logCameraCharacteristics: end");
    }

    /**
     * Closes the current {@link CameraDevice}.
     */
    private void closeCamera() {
        try {
            mCameraOpenCloseLock.acquire();
            if (null != mCaptureSession) {
                mCaptureSession.close();
                mCaptureSession = null;
            }
            if (null != mCameraDevice) {
                mCameraDevice.close();
                mCameraDevice = null;
            }
            if (null != mImageReader) {
                mImageReader.close();
                mImageReader = null;
            }
        } catch (InterruptedException e) {
            throw new RuntimeException("Interrupted while trying to lock camera closing.", e);
        } finally {
            mCameraOpenCloseLock.release();
        }
    }

    /**
     * Starts a background thread and its {@link Handler}.
     */
    private void startBackgroundThread() {
        mBackgroundThread = new HandlerThread("CameraBackground");
        mBackgroundThread.start();
        backgroundHandler = new Handler(mBackgroundThread.getLooper());
        mBackgroundThread2 = new HandlerThread("CameraBackground2");
        mBackgroundThread2.start();
        mBackgroundHandler2 = new Handler(mBackgroundThread2.getLooper());
    }

    /**
     * Stops the background thread and its {@link Handler}.
     */
    private void stopBackgroundThread() {
        mBackgroundThread.quitSafely();
        try {
            mBackgroundThread.join();
            mBackgroundThread = null;
            backgroundHandler = null;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        mBackgroundThread2.quitSafely();
        try {
            mBackgroundThread2.join();
            mBackgroundThread2 = null;
            mBackgroundHandler2 = null;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * TODO see https://stackoverflow.com/a/43564630/1200764
     * TODO see https://developer.android.com/reference/android/view/TextureView.html for getSurfaceTexture()
     * Creates a new {@link CameraCaptureSession} for camera preview.
     */
    private void createCameraPreviewSession() {
            Log.i("Camera2BasicFragment","createCameraPreviewSession(): start");
        try {
            SurfaceTexture previewTexture = mTextureView.getSurfaceTexture();
            assert previewTexture != null;

            // We configure the size of default buffer to be the size of camera preview we want.
            previewTexture.setDefaultBufferSize(imageSize.getWidth(), imageSize.getHeight());

            // This is the output Surface we need to start preview.
            Surface previewDisplaySurface = new Surface(previewTexture);

            // We set up a CaptureRequest.Builder with the output Surface.
            Log.i("Camera2BasicFragment","createCameraPreviewSession(): for mCameraDevice.getId()="+mCameraDevice.getId());
            mPreviewRequestBuilder = mCameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
            mPreviewRequestBuilder.addTarget(previewDisplaySurface);
            mPreviewRequestBuilder.addTarget(mImageReader.getSurface());            // TODO see https://stackoverflow.com/a/43564630/1200764

            // Here, we create a CameraCaptureSession for camera preview.

            /**********************************************************************************************/
            mCameraDevice.createCaptureSession(Arrays.asList(previewDisplaySurface, mImageReader.getSurface()),
                    new CameraCaptureSession.StateCallback() {

                        @Override
                        public void onConfigured(@NonNull CameraCaptureSession cameraCaptureSession) {
                            // The camera is already closed
                            if (null == mCameraDevice) {
                                return;
                            }
                            // When the session is ready, we start displaying the preview.
                            mCaptureSession = cameraCaptureSession;
                            try {
                                mPreviewRequestBuilder.set( // Auto focus should be continuous for camera preview.
                                        CaptureRequest.CONTROL_AF_MODE,
                                        CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE);

                                setAutoexposureToTargetRangeOfFPS(fpsRange);
                                setAutoFlash(mPreviewRequestBuilder); // Flash is automatically enabled when necessary.
                                mPreviewRequest = mPreviewRequestBuilder.build(); // Finally, we start displaying the camera preview.
                                mCaptureSession.setRepeatingRequest(
                                        mPreviewRequest,
                                        mCaptureCallback,
                                        backgroundHandler);
                            } catch (CameraAccessException e) {
                                e.printStackTrace();
                            }
                        }

                        @Override
                        public void onConfigureFailed(
                                @NonNull CameraCaptureSession cameraCaptureSession) {
                            showToast("Cofiguration process failed");   // output to screen
                        }
                    }
                    /**********************************************************************************************/
                    , null
            );
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }

    /**
     * Configures the necessary {@link android.graphics.Matrix} transformation to `mTextureView`.
     * This method should be called after the camera preview size is determined in
     * setUpCameraOutputs and also the size of `mTextureView` is fixed.
     *
     * @param viewWidth  The width of `mTextureView`
     * @param viewHeight The height of `mTextureView`
     */
    private void configureTransform(int viewWidth, int viewHeight) {
        Activity activity = getActivity();
        if (null == mTextureView || null == imageSize || null == activity) {
            return;
        }
        int rotation = activity.getWindowManager().getDefaultDisplay().getRotation();
        Matrix matrix = new Matrix();
        RectF viewRect = new RectF(0, 0, viewWidth, viewHeight);
        RectF bufferRect = new RectF(0, 0, imageSize.getHeight(), imageSize.getWidth());
        float centerX = viewRect.centerX();
        float centerY = viewRect.centerY();
        if (Surface.ROTATION_90 == rotation || Surface.ROTATION_270 == rotation) {
            bufferRect.offset(centerX - bufferRect.centerX(), centerY - bufferRect.centerY());
            matrix.setRectToRect(viewRect, bufferRect, Matrix.ScaleToFit.FILL);  // Set the matrix to the scale and translate values that map the source rectangle to the destination rectangle
            float scale = Math.max(
                    (float) viewHeight / imageSize.getHeight(),
                    (float) viewWidth / imageSize.getWidth());
            matrix.postScale(scale, scale, centerX, centerY);
            matrix.postRotate(90 * (rotation - 2), centerX, centerY);
        } else if (Surface.ROTATION_180 == rotation) {
            matrix.postRotate(180, centerX, centerY);
        }
        mTextureView.setTransform(matrix);
    }


    private void adjustFps() {
        if (dontHaveEnoughDataYet()) {
            Log.i("Camera2BasicFragment","adjustFps(): numRecordsToUse < countOfFpsAndThreads.size(): "+numRecordsToUse+" < "+countOfFpsAndThreads.size());
            return;
        }
        FPS.Change change = calculateFPSChangeAndClear();
        switch (change) {
            case DECREASE:
                fps--;
                fpsRange = new Range<Integer>(fps,fps);
                Log.i("Camera2BasicFragment","adjustFps(): DECREASE: now "+fpsRange);
                unlockFocusAndReturnToPreview();
                Log.i("Camera2BasicFragment","adjustFps(): DECREASE: after unlockFocusAndReturnToPreview.");
                break;
            case INCREASE:
                fps++;
                fpsRange = new Range<Integer>(fps,fps);
                Log.i("Camera2BasicFragment","adjustFps(): INCREASE: now "+fpsRange);
                unlockFocusAndReturnToPreview();
                Log.i("Camera2BasicFragment","adjustFps(): INCREASE: after unlockFocusAndReturnToPreview.");
                break;
            case NO_CHANGE:
                Log.i("Camera2BasicFragment","adjustFps(): NO_CHANGE: still "+fpsRange);
                break;
            default:
                Log.i("Camera2BasicFragment","adjustFps(): don't know what this value is : "+change.name());
                break;
        }
    }

    private FPS.Change calculateFPSChangeAndClear() {
        int[] fps_ = new int[countOfFpsAndThreads.size()];
        int[] concurrentThreads = new int[countOfFpsAndThreads.size()];
        int i_ = 0;
        for ( PerformanceMetric perf : countOfFpsAndThreads.values()) {
            fps_[i_] = perf.fps();
            concurrentThreads[i_] = perf.concurrentThreads();
            i_++;
        }
        countOfFpsAndThreads.clear();
        return FPS.calc(fps_,concurrentThreads,numRecordsToUse,targetFPS, MAX_CONCURRENT_THREADS);
    }

    private boolean dontHaveEnoughDataYet() {
        return numRecordsToUse > countOfFpsAndThreads.size();
    }

    /**
     * Initiate a still image capture.
     */
    private void takePicture() {
        Log.i("Camera2BasicFragment","takePicture()");
        lockFocusAndCaptureImage();
    }

    /**
     * Lock the focus as the first step for a still image capture.
     */
    private void lockFocusAndCaptureImage() {
        try {
            // This is how to tell the camera to lock focus.
            mPreviewRequestBuilder.set(
                    CaptureRequest.CONTROL_AF_TRIGGER,
                    CameraMetadata.CONTROL_AF_TRIGGER_START);
            // Tell #mCaptureCallback to wait for the lock.
            mState = STATE_WAITING_FOR_FOCUS_LOCK;
            mCaptureSession.capture(
                    mPreviewRequestBuilder.build(),
                    mCaptureCallback,
                    backgroundHandler);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }

    /**
     * Run the precapture sequence for capturing a still image. This method should be called when
     * we get a response in {@link #mCaptureCallback} from {@link #lockFocusAndCaptureImage()}.
     */
    private void runPrecaptureSequenceAndCaptureImage() {
        Log.i("Camera2BasicFragment","runPrecaptureSequenceAndCaptureImage()");
        try {
            // This is how to tell the camera to trigger.
            mPreviewRequestBuilder.set(
                    CaptureRequest.CONTROL_AE_PRECAPTURE_TRIGGER,
                    CaptureRequest.CONTROL_AE_PRECAPTURE_TRIGGER_START);
            // Tell #mCaptureCallback to wait for the precapture sequence to be set.
            mState = STATE_WAITING_FOR_EXPOSURE_TO_BE_PRECAPTURE;
            mCaptureSession.capture(
                    mPreviewRequestBuilder.build(),
                    mCaptureCallback,
                    backgroundHandler);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }

    /**
     * Capture a still picture. This method should be called when we get a response in
     * {@link #mCaptureCallback} from both {@link #lockFocusAndCaptureImage()}.
     */
    private void captureStillPicture() {
        try {
            final Activity activity = getActivity();
            if (null == activity || null == mCameraDevice) {
                return;
            }
            // This is the CaptureRequest.Builder that we use to take a picture.
            final CaptureRequest.Builder captureBuilder =
                    mCameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_STILL_CAPTURE);
            captureBuilder.addTarget(mImageReader.getSurface());

            // Use the same AE and AF modes as the preview.
            captureBuilder.set(
                    CaptureRequest.CONTROL_AF_MODE,
                    CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE);
            setAutoFlash(captureBuilder);

            // Orientation
            int rotation = activity.getWindowManager().getDefaultDisplay().getRotation();
            captureBuilder.set(CaptureRequest.JPEG_ORIENTATION, getOrientation(rotation));

            CameraCaptureSession.CaptureCallback CaptureCallback
                    = new CameraCaptureSession.CaptureCallback() {

                @Override
                public void onCaptureCompleted(@NonNull CameraCaptureSession session,
                                               @NonNull CaptureRequest request,
                                               @NonNull TotalCaptureResult result) {
                    showToast("Saved: " + mFile);
                    Log.d(TAG, mFile.toString());
                    unlockFocusAndReturnToPreview();
                }
            };

            mCaptureSession.stopRepeating();
            mCaptureSession.capture(captureBuilder.build(), CaptureCallback, null);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }

    /**
     * Retrieves the JPEG orientation from the specified screen rotation.
     *
     * @param rotation The screen rotation.
     * @return The JPEG orientation (one of 0, 90, 270, and 360)
     */
    private int getOrientation(int rotation) {
        // Sensor orientation is 90 for most devices, or 270 for some devices (eg. Nexus 5X)
        // We have to take that into account and rotate JPEG properly.
        // For devices with orientation of 90, we simply return our mapping from ORIENTATIONS.
        // For devices with orientation of 270, we need to rotate the JPEG 180 degrees.
        return (ORIENTATIONS.get(rotation) + mSensorOrientation + 270) % 360;
    }

    /**
     * Unlock the focus. This method should be called when still image capture sequence is
     * finished.
     */
    private void unlockFocusAndReturnToPreview() {
        try {
            // Reset the auto-focus trigger
            cancelAnyCurrentlyActiveAutofocusTrigger();
            setAutoexposureToTargetRangeOfFPS(fpsRange);

            setAutoFlash(mPreviewRequestBuilder);
            mCaptureSession.capture(
                    mPreviewRequestBuilder.build(),
                    mCaptureCallback,
                    backgroundHandler);

            // Return the camera to the 'normal' state of preview.
            mState = STATE_SHOWING_CAMERA_PREVIEW;
            mCaptureSession.setRepeatingRequest(
                    mPreviewRequest,
                    mCaptureCallback,
                    backgroundHandler);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }

    private void setAutoexposureToTargetRangeOfFPS(Range<Integer> rangeOfFPS) {
        Log.i("Camera2BasicFragment","setAutoexposureToTargetRangeOfFPS("+rangeOfFPS+")");
        mPreviewRequestBuilder.set( // Auto focus should be continuous for camera preview.
                CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE,
                rangeOfFPS);
    }

    private void cancelAnyCurrentlyActiveAutofocusTrigger() {
        mPreviewRequestBuilder.set(
                CaptureRequest.CONTROL_AF_TRIGGER,
                CameraMetadata.CONTROL_AF_TRIGGER_CANCEL);
    }

    @Override
    public void onClick(View view) {
        Log.i("Camera2BasicFragment","onClick(View view)");
        //        switch (view.getId()) {
        //            case R.id.picture: {
        //                takePicture();
        //                break;
        //            }
        //            case R.id.fps: {
        //                adjustFps();
        //                break;
        //            }
        //            case R.id.info: {
        //                Activity activity = getActivity();
        //                if (null != activity) {
        //                    new AlertDialog.Builder(activity)
        //                            .setMessage(R.string.intro_message)
        //                            .setPositiveButton(android.R.string.ok, null)
        //                            .show();
        //                }
        //                break;
        //            }
        //        }
    }

    private void setAutoFlash(CaptureRequest.Builder requestBuilder) {
        if (mFlashSupported) {
            requestBuilder.set(CaptureRequest.CONTROL_AE_MODE,
                    CaptureRequest.CONTROL_AE_MODE_ON_AUTO_FLASH);
        }
    }

    /**
     * Saves a JPEG {@link Image} into the specified {@link File}.
     */
    private static class ImageSaver implements Runnable {
        byte[] luminanceBytes;
        int imageWidth;
        int imageHeight;

        ImageSaver(Image image /*, File file*/) {
                Log.i("ImageSaver","ImageSaver(Image image)");
            long startTime = Calendar.getInstance().getTimeInMillis();
                Log.i("ImageSaver","ImageSaver(Image image): start = "+startTime);
            ByteBuffer luminanceBuffer = /*mImage*/image.getPlanes()[0].getBuffer();
                Log.i("ImageSaver","ImageSaver(Image image): after image.getPlanes()[0].getBuffer() in "+timeElapsed(startTime)+"ms");
            /*byte[]*/ luminanceBytes = new byte[luminanceBuffer.remaining()];  // buffer size: current position is zero, remaining() gives "the number of elements between the current position and the limit"
                Log.i("ImageSaver","ImageSaver(Image image): after luminanceBytes = new byte[luminanceBuffer.remaining()] in "+timeElapsed(startTime)+"ms");
            luminanceBuffer.get(luminanceBytes);                            // copy from buffer to bytes: get() "transfers bytes from this buffer into the given destination array"
            imageWidth = image.getWidth();
            imageHeight = image.getHeight();
                Log.i("ImageSaver","ImageSaver(Image image): end after "+timeElapsed(startTime)+"ms");
        }

        @Override
        public void run() {
                Log.i("ImageSaver","run()");
            long startTime = Calendar.getInstance().getTimeInMillis();
            Log.i("ImageSaver","run(): start = "+startTime);
            try {
                    Log.i("ImageSaver","run() : doing some work in the Try block");
                Random random = new Random();
                if ( 1== random.nextInt() ) {
                        Log.i("ImageSaver","run() : throwing an IOException at random while doing some work in the Try block");
                    throw new IOException("No particular reason: ImageSaver.run() : throwing an IOException at random while doing some work in the Try block");
                }
                // dummy image processing code - https://boofcv.org/index.php?title=Android_support
                long algorithmStepStartTime = Calendar.getInstance().getTimeInMillis();
                GrayU8 grayImage = new GrayU8(imageWidth,imageHeight);
                    Log.i("ImageSaver","run() : after constructing grayImage in "+timeElapsed(algorithmStepStartTime)+"ms");
                // from NV21 to gray scale
                algorithmStepStartTime = Calendar.getInstance().getTimeInMillis();
                ConvertNV21.nv21ToGray(luminanceBytes,imageWidth,imageHeight, grayImage);
                    Log.i("ImageSaver","run() : after converting nv21ToGray in "+timeElapsed(algorithmStepStartTime)+"ms");

                // start try detecting tags in the frame
                double BOOFCV_TAG_WIDTH= Hardcoding.BOOFCV_MARKER_SIZE_M; // TODO - tag size is a parameter
                int imageWidthInt = grayImage.getHeight(); // new Double(matGray.size().width).intValue();
                int imageHeightInt = grayImage.getWidth(); //new Double(matGray.size().height).intValue();detect
                    Log.i("ImageSaver","run() : image dimensions: "+imageWidthInt+" pixels wide, "+imageHeightInt+" pixels high");
                float imageWidthFloat =  (float)imageWidthInt; // new Double(matGray.size().width).intValue();
                float imageHeightFloat = (float)imageHeightInt; //new Double(matGray.size().height).intValue();
                float  focal_midpoint_pixels_x = imageWidthFloat/2.0f;
                float  focal_midpoint_pixels_y = imageHeightFloat/2.0f;

                double skew = 0.0;

                // TODO - 640 is now a magic number : it is the image width in pixels at the time of calibration of focal length
                // TODO - per-camera calibration using BoofCV calibration process
                float focal_length_in_pixels_x = 519.902859f * (imageWidthFloat/640.0f);  // TODO - for Samsung Galaxy S3s from /mnt/nixbig/ownCloud/project_AA1__1_1/results/2016_12_04_callibrate_in_ROS/calibrationdata_grey/ost.txt
                float focal_length_in_pixels_y = 518.952669f * (imageHeightFloat/480.0f);  // TODO - for Samsung Galaxy S3s from /mnt/nixbig/ownCloud/project_AA1__1_1/results/2016_12_04_callibrate_in_ROS/calibrationdata_grey/ost.txt


                    Log.i("ImageSaver","run() : config FactoryFiducial.squareBinary");
                FiducialDetector<GrayU8> detector = FactoryFiducial.squareBinary(
                        new ConfigFiducialBinary(BOOFCV_TAG_WIDTH),
                        ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 10),          // TODO - evaluate parameter - ?'radius'?  // TOOD - see other config thresholds
                        GrayU8.class);  // tag size,  type,  ?'radius'?
                //        detector.setLensDistortion(lensDistortion);
                    Log.i("ImageSaver","run() : config CameraPinhole pinholeModel");
                CameraPinhole pinholeModel = new CameraPinhole( // TODO - radial --> non-linearity with distance?
                        focal_length_in_pixels_x, focal_length_in_pixels_y,
                        skew,
                        focal_midpoint_pixels_x,focal_midpoint_pixels_y,
                        imageWidthInt,imageHeightInt);
                    Log.i("ImageSaver","run() : config LensDistortionNarrowFOV pinholeDistort");
                LensDistortionNarrowFOV pinholeDistort = new LensDistortionPinhole(pinholeModel);
                    Log.i("ImageSaver","run() : config detector.setLensDistortion(pinholeDistort)");
                detector.setLensDistortion(pinholeDistort);  // TODO TODO - do BoofCV calibration - but assume perfect pinhole camera for now

                // TODO - timing here  c[camera_num]-f[frameprocessed]
                long timeNow;
                    Log.i("ImageSaver","run() : start detector.detect(grayImage);");
                    Log.i("ImageSaver","run() : start detector.detect(grayImage) at "+Calendar.getInstance().getTimeInMillis());
                detector.detect(grayImage);
                    Log.i("ImageSaver","run() : after detector.detect(grayImage) in "+timeElapsed(startTime)+"ms");
                    Log.i("ImageSaver","run() : finished detector.detect(grayImage);");
                String logTag = "ImageSaver";
                for (int i = 0; i < detector.totalFound(); i++) {
                    timeNow = Calendar.getInstance().getTimeInMillis();
                    if( detector.hasUniqueID() ) {
                        long tag_id_long = detector.getId(i);
                            Log.i(logTag, "run() : tag detection "+i+" after detector.getId("+i+") = "+tag_id_long+" in " + timeElapsed(timeNow) + "ms");
                            Log.i(logTag, "run() : tag detection "+i+" after detector.getId("+i+") = "+tag_id_long+" in " + timeElapsed(startTime) + "ms");
                    } else {
                            Log.i(logTag, "run() : tag detection "+i+" has no id; detector.hasUniqueID() == false ");
                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
            } finally {
            }
        }

    }

    static private class TaskCompletionTimer {

        long threadCompletions = 1L;
        long previousInitiationTimeMs = -1;
        long previousExecutionStartTimeMs = -1;
        long previousExecutionEndTimeMs = -1;
        long overlapWithLastInitiation = -1;
        long overlapWithLastExecution = -1;
        long completionAfterPrevious = -1;
        long executionPeriod = -1;
        AtomicInteger concurrentThreadsExecuting = new AtomicInteger(0);

        private TaskCompletionTimer() {
        }

        static TaskCompletionTimer instance() {
            return new TaskCompletionTimer();
        }

        long overlapWithLastInitiation() {
            return overlapWithLastInitiation;
        }

        int incConcurrentThreadsExecuting() {
            return concurrentThreadsExecuting.incrementAndGet();
        }

        int decConcurrentThreadsExecuting() {
            return concurrentThreadsExecuting.decrementAndGet();
        }

        int concurrentThreadsExecuting() {
            return concurrentThreadsExecuting.intValue();
        }

        void completedTask(long executionThreadId, long initiationTimeMs, long executionStartTimeMs, long executionEndTimeMs) {
            decConcurrentThreadsExecuting();
            completionAfterPrevious = 0;
            if (threadCompletions>1) {
                overlapWithLastInitiation = executionEndTimeMs-previousInitiationTimeMs;
                overlapWithLastExecution = executionEndTimeMs-previousInitiationTimeMs;
                completionAfterPrevious = executionEndTimeMs-previousExecutionEndTimeMs;
                executionPeriod = executionEndTimeMs-executionStartTimeMs;
            }
            if (completionAfterPrevious > 0) {
                Log.i("completedTask","thread completion "+completionAfterPrevious+"ms after previous: "+overlapWithLastInitiation+"ms overlap with last started: thread "+executionThreadId+" was the "+threadCompletions+"th thread to complete at "+executionEndTimeMs+", started at "+executionStartTimeMs);
                previousInitiationTimeMs = initiationTimeMs;
                previousExecutionStartTimeMs = executionStartTimeMs;
                previousExecutionEndTimeMs = executionEndTimeMs;
            }
            threadCompletions++;
        }
    }


    private static final ThreadFactory sThreadFactory = new ThreadFactory() {
        private final AtomicInteger mCount = new AtomicInteger(1);

//        public Thread newThread(Runnable r) {
//            return new Thread(r, "AsyncTask #" + mCount.getAndIncrement());
//        }

        @Override       public Thread newThread(final Runnable runnable_) {
            Runnable wrapperRunnable = new Runnable() {
                @Override
                public void run() {
                    try {
                        android.os.Process.setThreadPriority(Process.THREAD_PRIORITY_FOREGROUND);
                    } catch (Throwable t) {

                    }
                    runnable_.run();
                }
            };
            return new Thread(wrapperRunnable);
        }
    };

    final static int THREADING_NUM_THREADS_CORE = 4; // 8
    final static int THREADING_NUM_THREADS_MAX  = 8; // 6 // 16
    static final int MAX_CONCURRENT_THREADS = THREADING_NUM_THREADS_MAX;  // Manual skip-if-over value
    final static int THREADING_QUEUE_SIZE = 256; //8; // 256; //64; // 6 // 16
    static final int KEEP_ALIVE_TIME = 5; // 30

    private static final BlockingQueue<Runnable> sPoolWorkQueue =
            new LinkedBlockingQueue<Runnable>(THREADING_QUEUE_SIZE);


    ThreadPoolExecutor threadPoolExecutor = new ThreadPoolExecutor(
            THREADING_NUM_THREADS_CORE, THREADING_NUM_THREADS_MAX
            , KEEP_ALIVE_TIME , TimeUnit.SECONDS,    // CORE_POOL_SIZE, MAXIMUM_POOL_SIZE, KEEP_ALIVE_SECONDS, TimeUnit.SECONDS,
            sPoolWorkQueue, sThreadFactory);


    /*
    TODO - see - https://stackoverflow.com/questions/25647881/android-asynctask-example-and-explanation
     */
    /**
     * Saves a JPEG {@link Image} into the specified {@link File}.
     */

    private class TaskHolder {
        byte[] luminanceBytes;
        int imageWidth;
        int imageHeight;

        long instantiationTimeMs = -1L;
        long executionThreadId = -1L;
        long executionStartTimeMs = -1L;
        long executionEndTimeMs = -1L;

        TaskHolder() {
        }

        TaskHolder(Image image, TaskCompletionTimer taskCompletionTimer_) {
            Log.i("TaskHolder","TaskHolder(Image image, TaskCompletionTimer taskCompletionTimer_)");
            long startTime = Calendar.getInstance().getTimeInMillis();
            setup(image, taskCompletionTimer_,startTime);
            Log.i("TaskHolder","ImageSaverAsyncTask(Image image, TaskCompletionTimer taskCompletionTimer_): end after "+timeElapsed(startTime)+"ms");
        }

        private void setup(Image image, TaskCompletionTimer taskCompletionTimer_, long startTime) {
            instantiationTimeMs = startTime;
            Log.i("TaskHolder","setup(Image image, TaskCompletionTimer taskCompletionTimer_): start = "+startTime);
            luminanceToBytes(image.getPlanes()[0], startTime);
            imageWidth = image.getWidth();
            imageHeight = image.getHeight();
            Log.i("TaskHolder","setup(Image image, TaskCompletionTimer taskCompletionTimer_): image dimensions: "+imageWidth+" pixels wide, "+imageHeight+" pixels high");
            taskCompletionTimer = taskCompletionTimer_;
        }

        private void luminanceToBytes(Image.Plane plane, long startTime) {
            ByteBuffer luminanceBuffer = /*mImage*/plane.getBuffer();
            Log.i("TaskHolder","luminanceToBytes(Image image): after image.getPlanes()[0].getBuffer() in "+timeElapsed(startTime)+"ms");
            /*byte[]*/
            luminanceBytes = new byte[luminanceBuffer.remaining()];  // buffer size: current position is zero, remaining() gives "the number of elements between the current position and the limit"
            Log.i("TaskHolder","luminanceToBytes(Image image): after luminanceBytes = new byte[luminanceBuffer.remaining()] in "+timeElapsed(startTime)+"ms");
            luminanceBuffer.get(luminanceBytes);                            // copy from buffer to bytes: get() "transfers bytes from this buffer into the given destination array"
            Log.i("TaskHolder","luminanceToBytes(Image image): after luminanceBuffer.get(luminanceBytes) in "+timeElapsed(startTime)+"ms");
        }


        private boolean  poseKnown   = false;

        private void updateLocationFromDetectedFeature(int tag_id, String logTagTag, Se3_F64 sensorToTargetViaTransform, Quaternion_F64 sensorToTargetViaTransformQuat) {
            if (!poseKnown) {
                long updateLocationFromDetectedFeatureStartTime = Calendar.getInstance().getTimeInMillis();
                rosThingy.updateLocationFromDetectedFeature(tag_id,
                        sensorToTargetViaTransform.getX(), sensorToTargetViaTransform.getY(), sensorToTargetViaTransform.getZ(),
                        sensorToTargetViaTransformQuat.x,sensorToTargetViaTransformQuat.y,sensorToTargetViaTransformQuat.z,sensorToTargetViaTransformQuat.w);
                //// TODO - timing here  c[camera_num]-f[frameprocessed]-i[iteration]-t[tagid]
                // long updateLocationFromDetectedFeatureStartTime = Calendar.getInstance().getTimeInMillis();
                Log.i(logTagTag, "ROS timing: rosThingy.updateLocationFromDetectedFeature took "+timeElapsed(updateLocationFromDetectedFeatureStartTime)+"ms");
                Log.i(logTagTag,"after localiseFromAFeatureClient.localiseFromAFeature");
            }
        }

        private void recordPerformance(long startTime, int fps) {
            countOfFpsAndThreads.put("p="+executionThreadId+",t="+startTime, new PerformanceMetric(fps, taskCompletionTimer.concurrentThreadsExecuting()));
            logOfFpsAndThreads.put("p="+executionThreadId+",t="+startTime, new PerformanceMetric(fps, taskCompletionTimer.concurrentThreadsExecuting()));
        }



        Long imageProcessingAndRosCalling() {
            /* Dev: part of robot visual model */
            HashMap<RobotId, List<DetectedTag>> robotsDetected = new HashMap<RobotId,List<DetectedTag>>();
            RobotId singleDummyRobotId = new RobotId(555); // new RobotId("dummy robot id");
            List<DetectedTag> robotFeatures = new ArrayList<DetectedTag>();
            List<DetectedTag> landmarkFeatures = new ArrayList<DetectedTag>();
            List<DetectedTag> quadFeatures = new ArrayList<DetectedTag>();
            /* end Dev: part of robot visual model */

            executionThreadId = Thread.currentThread().getId();
            String logTag = "ImgeSv_p="+executionThreadId;
            long procStartTime = Calendar.getInstance().getTimeInMillis();
            Log.i(logTag, "imageProcessingAndRosCalling(): start = " + procStartTime);
            if( MAX_CONCURRENT_THREADS < taskCompletionTimer.incConcurrentThreadsExecuting() ) {
                Log.i(logTag, "imageProcessingAndRosCalling(): stopping without processing: there are too many threads executing ");
                return new Long(0L);
            }
            executionStartTimeMs = procStartTime;
            try {
                Log.i(logTag, "imageProcessingAndRosCalling() : doing some work in the Try block: concurrentThreadsExecuting = "+taskCompletionTimer.concurrentThreadsExecuting());
                Log.i(logTag, "imageProcessingAndRosCalling() : doing some work in the Try block: Runtime.getRuntime().availableProcessors() = "+Runtime.getRuntime().availableProcessors());

                Random random = new Random();
                if (1 == random.nextInt()) {
                    Log.i(logTag, "imageProcessingAndRosCalling() : throwing an IOException at random while doing some work in the Try block");
                    throw new IOException("No particular reason: ImageSaver.doInBackground() : throwing an IOException at random while doing some work in the Try block");
                }
                // dummy image processing code - https://boofcv.org/index.php?title=Android_support
                long algorithmStepStartTime = 0L;
                algorithmStepStartTime = Calendar.getInstance().getTimeInMillis();
                GrayU8 grayImage = fetchAGrayImageToUse(imageWidth, imageHeight); //  new GrayU8(imageWidth, imageHeight);
                Log.i(logTag, "imageProcessingAndRosCalling() : after constructing grayImage in " + timeElapsed(algorithmStepStartTime) + "ms");
                // from NV21 to gray scale
                algorithmStepStartTime = Calendar.getInstance().getTimeInMillis();
                ConvertNV21.nv21ToGray(luminanceBytes, imageWidth, imageHeight, grayImage);
                Log.i(logTag, "imageProcessingAndRosCalling() : after converting nv21ToGray in " + timeElapsed(algorithmStepStartTime) + "ms");

                // start try detecting tags in the frame
                double BOOFCV_TAG_WIDTH = Hardcoding.BOOFCV_MARKER_SIZE_M;
                int imageWidthInt = grayImage.getHeight(); // new Double(matGray.size().width).intValue();
                int imageHeightInt = grayImage.getWidth(); //new Double(matGray.size().height).intValue();
                Log.i(logTag,"imageProcessingAndRosCalling() : image dimensions: "+imageWidthInt+" pixels wide, "+imageHeightInt+" pixels high");
                float imageWidthFloat = (float) imageWidthInt; // new Double(matGray.size().width).intValue();
                float imageHeightFloat = (float) imageHeightInt; //new Double(matGray.size().height).intValue();
                //                float focal_midpoint_pixels_x = imageWidthFloat * 0.5f;
                //                float focal_midpoint_pixels_y = imageHeightFloat * 0.5f;

                double skew = SKEW_PIXELS_AS_CALIBRATED;

                // TODO - 640 is now a magic number : it is the image width in pixels at the time of calibration of focal length
                float focal_length_in_pixels_x = FX_FOCAL_LENGTH_X_PIXELS_AS_CALIBRATED * (imageWidthFloat / IMAGE_WIDTH_PIXELS_AS_CALIBRATED);  // TODO - for Samsung Galaxy S3s from /mnt/nixbig/ownCloud/project_AA1__1_1/results/2016_12_04_callibrate_in_ROS/calibrationdata_grey/ost.txt
                float focal_length_in_pixels_y = FY_FOCAL_LENGTH_Y_PIXELS_AS_CALIBRATED * (imageHeightFloat / IMAGE_HEIGHT_PIXELS_AS_CALIBRATED);  // TODO - for Samsung Galaxy S3s from /mnt/nixbig/ownCloud/project_AA1__1_1/results/2016_12_04_callibrate_in_ROS/calibrationdata_grey/ost.txt
                float focal_midpoint_pixels_x = CX_FOCAL_MIDPOINT_X_PIXELS_AS_CALIBRATED * (imageWidthFloat / IMAGE_WIDTH_PIXELS_AS_CALIBRATED);  // TODO - for Samsung Galaxy S3s from /mnt/nixbig/ownCloud/project_AA1__1_1/results/2016_12_04_callibrate_in_ROS/calibrationdata_grey/ost.txt
                float focal_midpoint_pixels_y = CY_FOCAL_MIDPOINT_Y_PIXELS_AS_CALIBRATED * (imageHeightFloat / IMAGE_HEIGHT_PIXELS_AS_CALIBRATED);  // TODO - for Samsung Galaxy S3s from /mnt/nixbig/ownCloud/project_AA1__1_1/results/2016_12_04_callibrate_in_ROS/calibrationdata_grey/ost.txt
                float radial_1 = RADIAL_1_AS_CALIBRATED * 0.5F * ((imageWidthFloat / IMAGE_WIDTH_PIXELS_AS_CALIBRATED)+(imageHeightFloat / IMAGE_HEIGHT_PIXELS_AS_CALIBRATED));  // TODO is this right ??
                float radial_2 = RADIAL_2_AS_CALIBRATED * 0.5F * ((imageWidthFloat / IMAGE_WIDTH_PIXELS_AS_CALIBRATED)+(imageHeightFloat / IMAGE_HEIGHT_PIXELS_AS_CALIBRATED));  // TODO is this right ??

                Log.i(logTag, "imageProcessingAndRosCalling() : config FactoryFiducial.squareBinary");
                ConfigFiducialBinary config = new ConfigFiducialBinary(BOOFCV_TAG_WIDTH);
                boolean robust = true;
                int binaryThreshold_fixed = 50;
                ConfigThreshold configThreshold;
                if (robust) {
                    configThreshold = ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 10); // 6-pixel blocks of local-thresholded
                } else {
                    configThreshold = ConfigThreshold.fixed(binaryThreshold_fixed);         // image-wide: may be faster but may not cope with illumination variation - TODO - check vs FPS - make part of adaption process
                        // TODO - adaption process includes trading information through the service between smart cameras
                }
                FiducialDetector<GrayU8> detector = FactoryFiducial.squareBinary(config, configThreshold, GrayU8.class);
//                FiducialDetector<GrayU8> detector = FactoryFiducial.squareBinary(
//                        new ConfigFiducialBinary(BOOFCV_TAG_WIDTH),
//                        ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 10),          // TODO - evaluate parameter - ?'radius'?
//                        GrayU8.class);  // tag size,  type,  ?'radius'?
                //        detector.setLensDistortion(lensDistortion);
                Log.i(logTag, "imageProcessingAndRosCalling() : config CameraPinhole pinholeModel");
                CameraPinholeRadial pinholeModel = new CameraPinholeRadial(
                        focal_length_in_pixels_x, focal_length_in_pixels_y,
                        skew,
                        focal_midpoint_pixels_x, focal_midpoint_pixels_y,
                        imageWidthInt, imageHeightInt);
                pinholeModel.fsetRadial(radial_1,radial_2);
                    // TODO - .setRadial  - see if that compensates the nonlinear with distance - note that LensDistortionPinhole does not use the radial
                Log.i(logTag, "imageProcessingAndRosCalling() : config LensDistortionNarrowFOV pinholeDistort");
                LensDistortionNarrowFOV pinholeDistort = new LensDistortionRadialTangential(pinholeModel); //= new LensDistortionPinhole(pinholeModel);
                    // TODO - LensDistortionRadialTangential  - see if that compensates the nonlinear with distance
                Log.i(logTag, "imageProcessingAndRosCalling() : config detector.setLensDistortion(pinholeDistort)");
                detector.setLensDistortion(pinholeDistort);  // TODO - do BoofCV calibration - but assume perfect pinhole camera for now

                long timeNow = Calendar.getInstance().getTimeInMillis();
                Log.i(logTag, "imageProcessingAndRosCalling() : start detector.detect(grayImage) at " + timeNow);
                detector.detect(grayImage);
                Log.i(logTag, "imageProcessingAndRosCalling() : after detector.detect(grayImage) in " + timeElapsed(timeNow) + "ms: time since start = " + timeElapsed(procStartTime) + "ms");
                for (int detectionOrder_ = 0; detectionOrder_ < detector.totalFound(); detectionOrder_++) {
                    String logTagIteration = logTag+" c"+rosThingy.getCamNum()+"-detectionOrder_"+detectionOrder_;
                    timeNow = Calendar.getInstance().getTimeInMillis();
                    int tag_id = -1;
                    MarkerIdValidator isTagIdValid = new MarkerIdValidator(detector, detectionOrder_, tag_id).invoke();


                    tag_id = isTagIdValid.getTag_id();
                    if (!isTagIdValid.isValid()
                        ||
                        (
                            !Hardcoding.isPartOfRobotVisualModel(tag_id) && !Hardcoding.isALandmark(tag_id) && !Hardcoding.isAQuad(tag_id)
                            &&
                            !vosTaskSet.isThereAVisionTaskToExecute(tag_id,logTag)
                        )  ) {
                        //// todo - might want to do this processing and image processing in a background thread but then push the image into a short queue that the UI thread can pull the latest from
                        //// todo (cont) e.g. see https://stackoverflow.com/questions/19216893/android-camera-asynctask-with-preview-callback
                        //// todo (cont) note that BoofCV draws to Swing windows, which is nice because it's cross-platform
                        //// todo (cont) Processing or OpenGL or Unity might be better cross-platform choices
                        ////   todo - Processing - http://blog.blprnt.com/blog/blprnt/processing-android-mobile-app-development-made-very-easy  then  http://android.processing.org/  then https://www.mobileprocessing.org/cameras.html
                        ////
                        //// todo - for threaded, have a look at why this wouldn't work: https://stackoverflow.com/questions/14963773/android-asynctask-to-process-live-video-frames
                        ////    https://stackoverflow.com/questions/18183016/android-camera-frame-processing-with-multithreading?rq=1
                        ////    https://stackoverflow.com/questions/12215702/how-to-use-blocking-queue-to-process-camera-feed-in-background?noredirect=1&lq=1
                        ////    https://stuff.mit.edu/afs/sipb/project/android/docs/training/displaying-bitmaps/process-bitmap.html
                        ////    https://stuff.mit.edu/afs/sipb/project/android/docs/training/multiple-threads/index.html
                        ////    https://stuff.mit.edu/afs/sipb/project/android/docs/training/graphics/opengl/index.html
                        ////                          drawMarkerLocationOnDisplay_BoofCV(detector, i, FeatureModel.FEATURE_WITHOUT_3D_LOCATION);
                        continue;
                    }
                    String logTagTag = logTagIteration+"-t"+tag_id;

                    finishedUsingGrayImage(grayImage); grayImage = null;  // finished using the image, return it to the queue : cannot use it after this point

                    if( detector.hasUniqueID() ) {
                        long tag_id_long = detector.getId(detectionOrder_);
                        Log.i(logTag, "imageProcessingAndRosCalling() : tag detection "+detectionOrder_+" after detector.getId("+detectionOrder_+") = "+tag_id_long+" in " + timeElapsed(timeNow) + "ms : in " + timeElapsed(procStartTime) + "ms from start");
                        // if is for a current task, track it

                        // if is for a current task, report it
                    } else {
                        Log.i(logTag, "imageProcessingAndRosCalling() : tag detection "+detectionOrder_+" has no id; detector.hasUniqueID() == false ");
                        continue;
                    }



                    if( detector.is3D() ) {
                        algorithmStepStartTime = Calendar.getInstance().getTimeInMillis();
                        Log.i(logTagTag,"imageProcessingAndRosCalling() :start detector.getFiducialToCamera(detectionOrder_, targetToSensor_boofcvFrame);");
                        Se3_F64 targetToSensor_boofcvFrame = new Se3_F64();
                        detector.getFiducialToCamera(detectionOrder_, targetToSensor_boofcvFrame);
                        Log.i(logTagTag,"imageProcessingAndRosCalling() :after detector.getFiducialToCamera(detectionOrder_, targetToSensor_boofcvFrame);");

                        Vector3D_F64 transBoofCV_TtoS = targetToSensor_boofcvFrame.getTranslation();
                        Quaternion_F64 quatBoofCV_TtoS = new Quaternion_F64();
                        ConvertRotation3D_F64.matrixToQuaternion(targetToSensor_boofcvFrame.getR(), quatBoofCV_TtoS);

                            // testing 2017_08_23
                            double[] eulerZYZ=new double[]{0,0,0};
                            ConvertRotation3D_F64.matrixToEuler(targetToSensor_boofcvFrame.getR(), EulerType.ZYZ,eulerZYZ);
                            Log.i(logTagTag,"imageProcessingAndRosCalling() : testing 2017_08_23: eulerZYZ = "+eulerZYZ[0]+","+eulerZYZ[1]+","+eulerZYZ[2]);

                            Se3_F64 sensorToTarget_testing = null;
                            targetToSensor_boofcvFrame.invert(sensorToTarget_testing);
                            double[] eulerZYZ_fromInvert=new double[]{0,0,0};
                            ConvertRotation3D_F64.matrixToEuler(sensorToTarget_testing.getR(), EulerType.ZYZ,eulerZYZ_fromInvert);
                            Log.i(logTagTag,"imageProcessingAndRosCalling() : testing 2017_08_23: eulerZYZ_fromInvert = "+eulerZYZ_fromInvert[0]+","+eulerZYZ_fromInvert[1]+","+eulerZYZ_fromInvert[2]);



                        Log.i(logTagTag,"imageProcessingAndRosCalling() :3D Location: targetToSensor_boofcvFrame : BoofCV frame : x = " + transBoofCV_TtoS.getX() + ", y = " + transBoofCV_TtoS.getY() + ", z = " + transBoofCV_TtoS.getZ());
                        Log.i(logTagTag,"imageProcessingAndRosCalling() :3D Location: targetToSensor_boofcvFrame : BoofCV frame : qx = " + quatBoofCV_TtoS.x + ", qy = " + quatBoofCV_TtoS.y + ", qz = " + quatBoofCV_TtoS.z + ", qw = " + quatBoofCV_TtoS.w);
                            // take BoofCV coordinate convention for fiducials : X=left Y=up Z=forward
                            // ROS / ENU : East is X and forward, North is Y and left and Up is Z and Up
                            // NED       : Notrht is X and
//                          Vector3D_F64

                        ConvertRotation3D_F64.matrixToQuaternion(targetToSensor_boofcvFrame.getR(), quatBoofCV_TtoS);
                        DenseMatrix64F transformation_fromBoofCVFiducialTagToSensor_toRobotSensorToTag
                                = new DenseMatrix64F(new double[][]{
                                {  0.0 ,  0.0 , -1.0 } ,
                                { -1.0 ,  0.0 ,  0.0 } ,
                                {  0.0 , +1.0 ,  0.0 } });
                        Se3_F64 sensorToTargetViaTransform = new Se3_F64();
                        DenseMatrix64F sensorToTargetViaTransformRot = CommonOps.identity(3);
                        CommonOps.mult(transformation_fromBoofCVFiducialTagToSensor_toRobotSensorToTag,targetToSensor_boofcvFrame.getR(),sensorToTargetViaTransformRot);
                        sensorToTargetViaTransform.setRotation(sensorToTargetViaTransformRot);
                        sensorToTargetViaTransform.setTranslation(transBoofCV_TtoS.getZ(), -1.0*transBoofCV_TtoS.getX(), -1.0*transBoofCV_TtoS.getY());
                        Quaternion_F64 sensorToTargetViaTransformQuat = new Quaternion_F64();
                        ConvertRotation3D_F64.matrixToQuaternion(sensorToTargetViaTransformRot, sensorToTargetViaTransformQuat);

                        double[] eulerBefore=new double[]{0,0,0};
                        ConvertRotation3D_F64.matrixToEuler(sensorToTargetViaTransformRot, EulerType.YXY,eulerBefore);
                        double[] eulerAfter = new double[] { eulerBefore[0], -1.0*eulerBefore[1], eulerBefore[2] };  // robot+Z+Y+Z = boof+Y-X+Y
                        ConvertRotation3D_F64.eulerToMatrix(EulerType.ZYZ, eulerAfter[0],   eulerAfter[1],      eulerAfter[2],    sensorToTargetViaTransformRot);
                        sensorToTargetViaTransform.setRotation(sensorToTargetViaTransformRot);
                        ConvertRotation3D_F64.matrixToQuaternion(sensorToTargetViaTransformRot, sensorToTargetViaTransformQuat);

                        //// TODO - timing here  c[camera_num]-f[frameprocessed]-detectionOrder_[iteration]-t[tagid]
                        Log.i(logTagTag,"imageProcessingAndRosCalling() : after applying transformations");
                        int tag_id_reported = MARKER_OFFSET_INT+tag_id;

                        long reportDetectedFeatureStartTime = Calendar.getInstance().getTimeInMillis();
                        Log.i(logTagTag,"imageProcessingAndRosCalling() : after calculating transformation to tag in "+timeElapsed(algorithmStepStartTime)+"ms");
                        rosThingy.reportDetectedFeature(tag_id_reported,
                                sensorToTargetViaTransform.getX(), sensorToTargetViaTransform.getY(), sensorToTargetViaTransform.getZ(),
                                sensorToTargetViaTransformQuat.x,sensorToTargetViaTransformQuat.y,sensorToTargetViaTransformQuat.z,sensorToTargetViaTransformQuat.w);
                        Log.i(logTagTag, "imageProcessingAndRosCalling() : ROS timing: rosThingy.reportDetectedFeature took "+timeElapsed(reportDetectedFeatureStartTime)+"ms");
                        System.out.println("imageProcessingAndRosCalling() : 3D Location: reporting tag_id "+tag_id_reported+" as : x = " + transBoofCV_TtoS.getX() + ", y = " + transBoofCV_TtoS.getY() + ", z = " + transBoofCV_TtoS.getZ());
                        System.out.println("imageProcessingAndRosCalling() : 3D Location: reporting tag_id "+tag_id_reported+" as : qx = " + quatBoofCV_TtoS.x + ", qy = " + quatBoofCV_TtoS.y + ", qz = " + quatBoofCV_TtoS.z + ", qw = " + quatBoofCV_TtoS.w);


                        /* Dev: part of robot visual model */
                        robotsDetected.put(singleDummyRobotId,robotFeatures);
                        Point2D_F64 locationPixel = new Point2D_F64();
                        detector.getImageLocation(detectionOrder_, locationPixel);        // pixel location in input image
                        boolean used=false;
                        if(Hardcoding.isPartOfRobotVisualModel(tag_id)) {
                            DetectedTag detectedTag = new DetectedTag(tag_id,sensorToTargetViaTransform,sensorToTargetViaTransformQuat);
                            robotFeatures.add(detectedTag);
                            used=true;
                            Log.i(logTagTag,"imageProcessingAndRosCalling() :isPartOfRobotVisualModel TAG - tag_id "+tag_id+" - 2D Image Location = "+locationPixel);
                        }
                        if(Hardcoding.isALandmark(tag_id)) {
                            DetectedTag detectedTag = new DetectedTag(tag_id, sensorToTargetViaTransform, locationPixel);
                            landmarkFeatures.add(detectedTag);
                            if(used) {Log.w(logTagTag,"imageProcessingAndRosCalling() :looks like tag_id"+tag_id+" is being used more than once");}
                            used=true;
                            Log.i(logTagTag, "imageProcessingAndRosCalling() :isALandmark TAG - tag_id " + tag_id + " landmarkFeatures.size()=" + landmarkFeatures.size() + " - 2D Image Location = " + locationPixel);
                        }
                        if(Hardcoding.isAQuad(tag_id)) {
                            DetectedTag detectedTag = new DetectedTag(tag_id, sensorToTargetViaTransform, locationPixel);
                            quadFeatures.add(detectedTag);
                            if(used) {Log.w(logTagTag,"imageProcessingAndRosCalling() :looks like tag_id"+tag_id+" is being used more than once");}
                            used=true;
                            Log.i(logTagTag, "imageProcessingAndRosCalling() :isAQuad TAG - tag_id " + tag_id + " quadFeatures.size()=" + quadFeatures.size() + " - 2D Image Location = " + locationPixel);
                        }
                        if(!used){ // not part of something that we are looking for, so ignore
                            Log.i(logTagTag,"imageProcessingAndRosCalling() :IGNORING TAG - not part of robot visual model - tag_id "+tag_id+" - 2D Image Location = "+locationPixel);
                            continue;
                        }
                        /* end Dev: part of robot visual model */

                        //// TODO - timing here  c[camera_num]-f[frameprocessed]-detectionOrder_[iteration]-t[tagid]
                        Log.i(logTagTag,"imageProcessingAndRosCalling() :after detectedFeaturesClient.reportDetectedFeature");
                        updateLocationFromDetectedFeature(tag_id, logTagTag, sensorToTargetViaTransform, sensorToTargetViaTransformQuat);
                        //                            variousUnusedAttemptsAtCoordinateSystemCorrection();

                    } else {  // 3D info not available for tag/marker
                        //                            drawMarkerLocationOnDisplay_BoofCV(detector, detectionOrder_, FeatureModel.FEATURE_WITHOUT_3D_LOCATION);
                    }
                }
                if(quadFeatures.size() >=4 ) {
                    Log.i(logTag,"imageProcessingAndRosCalling() :quadFeatures: enough quadFeatures found in image to estimate camera pose: quadFeatures.size()="+quadFeatures.size());
                    //                        Se3_F64 estimateCameraPoseFrom3D2DPointMatches(CameraPinholeRadial cameraDistortionCoefficients, int numPointsToUse, double[] worldX, double[] worldY, double[] worldZ, double[] pixelsX, double[] pixelsY);
                    CameraPinholeRadial cameraIntrinsics = new CameraPinholeRadial(
                            focal_length_in_pixels_x, focal_length_in_pixels_y,
                            skew,
                            focal_midpoint_pixels_x, focal_midpoint_pixels_y,
                            imageWidthInt, imageHeightInt);


                    Se3_F64 cameraPose = updatePoseEstimate(cameraIntrinsics,quadFeatures);
                    int numElementsInRot = cameraPose.R.getNumElements();
                    Vector3D_F64 translation = cameraPose.T;
                    Log.i(TAG, "imageProcessingAndRosCalling() :quadFeatures: cameraPose: " + /*" rotation numElements=" + numElementsInRot + ", rotation=" + cameraPose.R.toString() + */", translation =" + translation.toString());

                } else {
                    Log.i(logTag,"imageProcessingAndRosCalling() :quadFeatures: not enough quadFeatures found in image: quadFeatures.size()="+quadFeatures.size());
                }

            } catch (IOException e) {
                e.printStackTrace();
            } finally {
                long timeNow = Calendar.getInstance().getTimeInMillis();
                long timeElapsed = timeNow - procStartTime;
                int fps = (int) (1.0/(timeElapsed/1000.0));
                recordPerformance(procStartTime, fps);
            }
            executionEndTimeMs = Calendar.getInstance().getTimeInMillis();
            return new Long(0L);
        }



        /***************************************************************/

        private HashMap<Integer, double[]> fixedLandmarks;


        private Se3_F64 updatePoseEstimate(final CameraPinholeRadial cameraIntrinsics_, final List<DetectedTag> landmarkFeatures) {
            loadLandmarkFeaturesOnce();

            double[] worldX = new double[landmarkFeatures.size()];
            double[] worldY = new double[landmarkFeatures.size()];
            double[] worldZ = new double[landmarkFeatures.size()];
            double[] pixelsX = new double[landmarkFeatures.size()];
            double[] pixelsY = new double[landmarkFeatures.size()];
            int i_ = 0;
            for (DetectedTag tag:
                    landmarkFeatures) {
                double[] worldCoordinates = fixedLandmarks.get(tag.getTag_id());
                if(null != worldCoordinates) {
                    Point2D_F64 pixelLocation = tag.getLocationPixel();
                    worldX[i_] = worldCoordinates[0];
                    worldY[i_] = worldCoordinates[1];
                    worldZ[i_] = worldCoordinates[2];
                    pixelsX[i_] = pixelLocation.getX();  //
                    pixelsY[i_] = pixelLocation.getY();
                    i_++;
                }
            }
            PoseFrom3D2DPointMatches estimator = new LocalisePnP_BoofCV();
//            return estimator.estimateCameraPoseFrom3D2DPointMatches(
//                    cameraIntrinsics_,//CameraIntrinsics.exampleCameraPinholeRadial(),  /*  TODO - HARDCODING in here */
//                    landmarkFeatures.size(), worldX, worldY, worldZ, pixelsX, pixelsY);
            return estimator.estimateCameraPoseQuad(cameraIntrinsics_,worldX, worldY, worldZ, pixelsX, pixelsY);
        }

        public static final int FOUR_POINTS_REQUIRED_FOR_PNP = 4;

        private final LandmarkFeatureLoader landmarkFeatureLoader = new LandmarkFeatureLoader();  ////  TODO - list of tags and sizes, and tag-groups and sizes

        private void loadLandmarkFeaturesOnce() {
        /*  TODO - HARDCODING  */
            if(null == fixedLandmarks) {fixedLandmarks = landmarkFeatureLoader.loadLandmarkFeatures();}
        /*  TODO - HARDCODING  */
        }
        /***************************************************************/

    }


    /**********************************************************************************************/


    private class ImageSaverAsyncTask extends AsyncTask<Void, Void, Long> { //parameter array type, progress type, return type
        byte[] luminanceBytes;
        int imageWidth;
        int imageHeight;

        long instantiationTimeMs = -1L;
        long executionThreadId = -1L;
        long executionStartTimeMs = -1L;
        long executionEndTimeMs = -1L;

        TaskHolder taskHolder = null;

        public ImageSaverAsyncTask(Image image, TaskCompletionTimer taskCompletionTimer_) {
            super();
            taskHolder = new TaskHolder(image, taskCompletionTimer_);
        }

        @Override
        protected Long doInBackground(Void... params) { //(Image...  images) {
            return taskHolder.imageProcessingAndRosCalling();


        }

        @NonNull
        private Long imageProcessingAndRosCalling() {
            return new TaskHolderMethodObject().invoke();

        }


        /***************************************************************/

        private HashMap<Integer, double[]> fixedLandmarks;


        private Se3_F64 updatePoseEstimate(final CameraPinholeRadial cameraIntrinsics_, final List<DetectedTag> landmarkFeatures) {
            loadLandmarkFeaturesOnce();

            double[] worldX = new double[landmarkFeatures.size()];
            double[] worldY = new double[landmarkFeatures.size()];
            double[] worldZ = new double[landmarkFeatures.size()];
            double[] pixelsX = new double[landmarkFeatures.size()];
            double[] pixelsY = new double[landmarkFeatures.size()];
            int i_ = 0;
            for (DetectedTag tag:
                    landmarkFeatures) {
                double[] worldCoordinates = fixedLandmarks.get(tag.getTag_id());
                if(null != worldCoordinates) {
                    Point2D_F64 pixelLocation = tag.getLocationPixel();
                    worldX[i_] = worldCoordinates[0];
                    worldY[i_] = worldCoordinates[1];
                    worldZ[i_] = worldCoordinates[2];
                    pixelsX[i_] = pixelLocation.getX();  //
                    pixelsY[i_] = pixelLocation.getY();
                    i_++;
                }
            }
            PoseFrom3D2DPointMatches estimator = new LocalisePnP_BoofCV();
//            return estimator.estimateCameraPoseFrom3D2DPointMatches(
//                    cameraIntrinsics_,//CameraIntrinsics.exampleCameraPinholeRadial(),  /*  TODO - HARDCODING in here */
//                    landmarkFeatures.size(), worldX, worldY, worldZ, pixelsX, pixelsY);
            return estimator.estimateCameraPoseQuad(cameraIntrinsics_,worldX, worldY, worldZ, pixelsX, pixelsY);
        }

        public static final int FOUR_POINTS_REQUIRED_FOR_PNP = 4;
        private final LandmarkFeatureLoader landmarkFeatureLoader = new LandmarkFeatureLoader();  ////  TODO - list of tags and sizes, and tag-groups and sizes
        private void loadLandmarkFeaturesOnce() {
        /*  TODO - HARDCODING  */
            if(null == fixedLandmarks) {fixedLandmarks = landmarkFeatureLoader.loadLandmarkFeatures();}
        /*  TODO - HARDCODING  */
        }
        /***************************************************************/


        private boolean  poseKnown   = false;
        private void updateLocationFromDetectedFeature(int tag_id, String logTagTag, Se3_F64 sensorToTargetViaTransform, Quaternion_F64 sensorToTargetViaTransformQuat) {
            if (!poseKnown) {
                long updateLocationFromDetectedFeatureStartTime = Calendar.getInstance().getTimeInMillis();
                rosThingy.updateLocationFromDetectedFeature(tag_id,
                        sensorToTargetViaTransform.getX(), sensorToTargetViaTransform.getY(), sensorToTargetViaTransform.getZ(),
                        sensorToTargetViaTransformQuat.x,sensorToTargetViaTransformQuat.y,sensorToTargetViaTransformQuat.z,sensorToTargetViaTransformQuat.w);
                //// TODO - timing here  c[camera_num]-f[frameprocessed]-i[iteration]-t[tagid]
                // long updateLocationFromDetectedFeatureStartTime = Calendar.getInstance().getTimeInMillis();
                Log.i(logTagTag, "ROS timing: rosThingy.updateLocationFromDetectedFeature took "+timeElapsed(updateLocationFromDetectedFeatureStartTime)+"ms");
                Log.i(logTagTag,"after localiseFromAFeatureClient.localiseFromAFeature");
            }
        }

        private void recordPerformance(long startTime, int fps) {
            countOfFpsAndThreads.put("p="+executionThreadId+",t="+startTime, new PerformanceMetric(fps, taskCompletionTimer.concurrentThreadsExecuting()));
            logOfFpsAndThreads.put("p="+executionThreadId+",t="+startTime, new PerformanceMetric(fps, taskCompletionTimer.concurrentThreadsExecuting()));
        }


        @Override
        protected void onPreExecute() {
            super.onPreExecute();
        }

        @Override
        protected void onProgressUpdate(Void... values) {
            super.onProgressUpdate(values);
        }

        @Override
        protected void onPostExecute(Long result) {
            super.onPostExecute(result);
            taskCompletionTimer.completedTask(executionThreadId, instantiationTimeMs, executionStartTimeMs, executionEndTimeMs);
        }

        private class TaskHolderMethodObject {
            public Long invoke() {
    /* Dev: part of robot visual model */
                HashMap<RobotId, List<DetectedTag>> robotsDetected = new HashMap<RobotId,List<DetectedTag>>();
                RobotId singleDummyRobotId = new RobotId(555); // new RobotId("dummy robot id");
                List<DetectedTag> robotFeatures = new ArrayList<DetectedTag>();
                List<DetectedTag> landmarkFeatures = new ArrayList<DetectedTag>();
                List<DetectedTag> quadFeatures = new ArrayList<DetectedTag>();
            /* end Dev: part of robot visual model */

                executionThreadId = Thread.currentThread().getId();
                String logTag = "ImgeSv_p="+executionThreadId;
                long procStartTime = Calendar.getInstance().getTimeInMillis();
                Log.i(logTag, "imageProcessingAndRosCalling(): start = " + procStartTime);
                if( MAX_CONCURRENT_THREADS < taskCompletionTimer.incConcurrentThreadsExecuting() ) {
                    Log.i(logTag, "invoke(): stopping without processing: there are too many threads executing ");
                    return new Long(0L);
                }
                executionStartTimeMs = procStartTime;
                try {
                        Log.i(logTag, "invoke() : doing some work in the Try block: concurrentThreadsExecuting = "+taskCompletionTimer.concurrentThreadsExecuting());
                        Log.i(logTag, "invoke() : doing some work in the Try block: Runtime.getRuntime().availableProcessors() = "+Runtime.getRuntime().availableProcessors());

                    Random random = new Random();
                    if (1 == random.nextInt()) {
                        Log.i(logTag, "invoke() : throwing an IOException at random while doing some work in the Try block");
                        throw new IOException("No particular reason: ImageSaver.doInBackground() : throwing an IOException at random while doing some work in the Try block");
                    }
                    // dummy image processing code - https://boofcv.org/index.php?title=Android_support
                        long algorithmStepStartTime = 0L;
                        algorithmStepStartTime = Calendar.getInstance().getTimeInMillis();
                    GrayU8 grayImage = fetchAGrayImageToUse(imageWidth, imageHeight); //  new GrayU8(imageWidth, imageHeight);
                        Log.i(logTag, "invoke() : after constructing grayImage in " + timeElapsed(algorithmStepStartTime) + "ms");
                    // from NV21 to gray scale
                        algorithmStepStartTime = Calendar.getInstance().getTimeInMillis();
                    ConvertNV21.nv21ToGray(luminanceBytes, imageWidth, imageHeight, grayImage);
                        Log.i(logTag, "invoke() : after converting nv21ToGray in " + timeElapsed(algorithmStepStartTime) + "ms");

                    // start try detecting tags in the frame
                    double BOOFCV_TAG_WIDTH = Hardcoding.BOOFCV_MARKER_SIZE_M;
                    int imageWidthInt = grayImage.getHeight(); // new Double(matGray.size().width).intValue();
                    int imageHeightInt = grayImage.getWidth(); //new Double(matGray.size().height).intValue();
                        Log.i(logTag,"invoke() : image dimensions: "+imageWidthInt+" pixels wide, "+imageHeightInt+" pixels high");
                    float imageWidthFloat = (float) imageWidthInt; // new Double(matGray.size().width).intValue();
                    float imageHeightFloat = (float) imageHeightInt; //new Double(matGray.size().height).intValue();
                    float focal_midpoint_pixels_x = imageWidthFloat / 2.0f;
                    float focal_midpoint_pixels_y = imageHeightFloat / 2.0f;

                    double skew = SKEW_PIXELS_AS_CALIBRATED;

                    // TODO - 640 is now a magic number : it is the image width in pixels at the time of calibration of focal length
                    float focal_length_in_pixels_x = FX_FOCAL_LENGTH_X_PIXELS_AS_CALIBRATED * (imageWidthFloat / IMAGE_WIDTH_PIXELS_AS_CALIBRATED);  // TODO - for Samsung Galaxy S3s from /mnt/nixbig/ownCloud/project_AA1__1_1/results/2016_12_04_callibrate_in_ROS/calibrationdata_grey/ost.txt
                    float focal_length_in_pixels_y = FY_FOCAL_LENGTH_Y_PIXELS_AS_CALIBRATED * (imageHeightFloat / IMAGE_HEIGHT_PIXELS_AS_CALIBRATED);  // TODO - for Samsung Galaxy S3s from /mnt/nixbig/ownCloud/project_AA1__1_1/results/2016_12_04_callibrate_in_ROS/calibrationdata_grey/ost.txt
                    float cx_focal_centre_in_pixels_x = CX * (imageWidthFloat / IMAGE_WIDTH_PIXELS_AS_CALIBRATED);
                    float cy_focal_centre_in_pixels_y = CY * (imageHeightFloat / IMAGE_HEIGHT_PIXELS_AS_CALIBRATED);
                    float radial_1 = RADIAL_1_AS_CALIBRATED * 0.5F * ((imageWidthFloat / IMAGE_WIDTH_PIXELS_AS_CALIBRATED)+(imageHeightFloat / IMAGE_HEIGHT_PIXELS_AS_CALIBRATED));  // TODO is this right ??
                    float radial_2 = RADIAL_2_AS_CALIBRATED * 0.5F * ((imageWidthFloat / IMAGE_WIDTH_PIXELS_AS_CALIBRATED)+(imageHeightFloat / IMAGE_HEIGHT_PIXELS_AS_CALIBRATED));  // TODO is this right ??


                            Log.i(logTag, "invoke() : config FactoryFiducial.squareBinary");
                    FiducialDetector<GrayU8> detector = FactoryFiducial.squareBinary(
                            new ConfigFiducialBinary(BOOFCV_TAG_WIDTH),
                            ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 10),          // TODO - evaluate parameter - ?'radius'? - size of area adaptive threshold is calculated over
                            GrayU8.class);  // tag size,  type,  ?'radius'?
                    //        detector.setLensDistortion(lensDistortion);
                    Log.i(logTag, "invoke() : config CameraPinhole pinholeModel");
                    CameraPinholeRadial pinholeModel = new CameraPinholeRadial(
                            focal_length_in_pixels_x, focal_length_in_pixels_y,
                            skew,
                            cx_focal_centre_in_pixels_x, cy_focal_centre_in_pixels_y,
                            imageWidthInt, imageHeightInt);
                    pinholeModel.fsetRadial(radial_1,radial_2);
                    Log.i(logTag, "invoke() : config LensDistortionNarrowFOV pinholeDistort");
                    LensDistortionNarrowFOV pinholeDistort = new LensDistortionPinhole(pinholeModel);
                    Log.i(logTag, "invoke() : config detector.setLensDistortion(pinholeDistort)");
                    detector.setLensDistortion(pinholeDistort);  // TODO - do BoofCV calibration - but assume perfect pinhole camera for now

                    // TODO - timing here  c[camera_num]-f[frameprocessed]
                    long timeNow = Calendar.getInstance().getTimeInMillis();
                        Log.i(logTag, "invoke() : start detector.detect(grayImage) at " + timeNow);
                    detector.detect(grayImage);
                        Log.i(logTag, "invoke() : after detector.detect(grayImage) in " + timeElapsed(timeNow) + "ms: time since start = " + timeElapsed(procStartTime) + "ms");
                    for (int detectionOrder_ = 0; detectionOrder_ < detector.totalFound(); detectionOrder_++) {
                        String logTagIteration = logTag+" c"+rosThingy.getCamNum()+"-detectionOrder_"+detectionOrder_;
                        timeNow = Calendar.getInstance().getTimeInMillis();
                        int tag_id = -1;
                        MarkerIdValidator isTagIdValid = new MarkerIdValidator(detector, detectionOrder_, tag_id).invoke();


                        if (!isTagIdValid.isValid()) {
    //// todo - might want to do this processing and image processing in a background thread but then push the image into a short queue that the UI thread can pull the latest from
    //// todo (cont) e.g. see https://stackoverflow.com/questions/19216893/android-camera-asynctask-with-preview-callback
    //// todo (cont) note that BoofCV draws to Swing windows, which is nice because it's cross-platform
    //// todo (cont) Processing or OpenGL or Unity might be better cross-platform choices
    ////   todo - Processing - http://blog.blprnt.com/blog/blprnt/processing-android-mobile-app-development-made-very-easy  then  http://android.processing.org/  then https://www.mobileprocessing.org/cameras.html
    ////
    //// todo - for threaded, have a look at why this wouldn't work: https://stackoverflow.com/questions/14963773/android-asynctask-to-process-live-video-frames
    ////    https://stackoverflow.com/questions/18183016/android-camera-frame-processing-with-multithreading?rq=1
    ////    https://stackoverflow.com/questions/12215702/how-to-use-blocking-queue-to-process-camera-feed-in-background?noredirect=1&lq=1
    ////    https://stuff.mit.edu/afs/sipb/project/android/docs/training/displaying-bitmaps/process-bitmap.html
    ////    https://stuff.mit.edu/afs/sipb/project/android/docs/training/multiple-threads/index.html
    ////    https://stuff.mit.edu/afs/sipb/project/android/docs/training/graphics/opengl/index.html
    ////                          drawMarkerLocationOnDisplay_BoofCV(detector, i, FeatureModel.FEATURE_WITHOUT_3D_LOCATION);
                            continue;
                        }
                        tag_id = isTagIdValid.getTag_id();
                        String logTagTag = logTagIteration+"-t"+tag_id;

                        finishedUsingGrayImage(grayImage); grayImage = null;  // finished using the image, return it to the queue : cannot use it after this point

                        if( detector.hasUniqueID() ) {
                            long tag_id_long = detector.getId(detectionOrder_);
                                Log.i(logTag, "invoke() : tag detection "+detectionOrder_+" after detector.getId("+detectionOrder_+") = "+tag_id_long+" in " + timeElapsed(timeNow) + "ms : in " + timeElapsed(procStartTime) + "ms from start");
                            // if is for a current task, track it

                            // if is for a current task, report it
                        } else {
                                Log.i(logTag, "invoke() : tag detection "+detectionOrder_+" has no id; detector.hasUniqueID() == false ");
                            continue;
                        }


                        if( detector.is3D() ) {
                            Log.i(logTagTag,"start detector.getFiducialToCamera(detectionOrder_, targetToSensor_boofcvFrame);");
                            Se3_F64 targetToSensor_boofcvFrame = new Se3_F64();
                            detector.getFiducialToCamera(detectionOrder_, targetToSensor_boofcvFrame);
                            Log.i(logTagTag,"after detector.getFiducialToCamera(detectionOrder_, targetToSensor_boofcvFrame);");

                            Vector3D_F64 transBoofCV_TtoS = targetToSensor_boofcvFrame.getTranslation();
                            Quaternion_F64 quatBoofCV_TtoS = new Quaternion_F64();
                            ConvertRotation3D_F64.matrixToQuaternion(targetToSensor_boofcvFrame.getR(), quatBoofCV_TtoS);
                            System.out.println("3D Location: targetToSensor_boofcvFrame : BoofCV frame : x = " + transBoofCV_TtoS.getX() + ", y = " + transBoofCV_TtoS.getY() + ", z = " + transBoofCV_TtoS.getZ());
                            System.out.println("3D Location: targetToSensor_boofcvFrame : BoofCV frame : qx = " + quatBoofCV_TtoS.x + ", qy = " + quatBoofCV_TtoS.y + ", qz = " + quatBoofCV_TtoS.z + ", qw = " + quatBoofCV_TtoS.w);

                            ConvertRotation3D_F64.matrixToQuaternion(targetToSensor_boofcvFrame.getR(), quatBoofCV_TtoS);
                            DenseMatrix64F transformation_fromBoofCVFiducialTagToSensor_toRobotSensorToTag
                                    = new DenseMatrix64F(new double[][]{
                                    {  0.0 ,  0.0 , -1.0 } ,
                                    { -1.0 ,  0.0 ,  0.0 } ,
                                    {  0.0 , +1.0 ,  0.0 } });
                            Se3_F64 sensorToTargetViaTransform = new Se3_F64();
                            DenseMatrix64F sensorToTargetViaTransformRot = CommonOps.identity(3);
                            CommonOps.mult(transformation_fromBoofCVFiducialTagToSensor_toRobotSensorToTag,targetToSensor_boofcvFrame.getR(),sensorToTargetViaTransformRot);
                            sensorToTargetViaTransform.setRotation(sensorToTargetViaTransformRot);
                            sensorToTargetViaTransform.setTranslation(transBoofCV_TtoS.getZ(), -1.0*transBoofCV_TtoS.getX(), -1.0*transBoofCV_TtoS.getY());
                            Quaternion_F64 sensorToTargetViaTransformQuat = new Quaternion_F64();
                            ConvertRotation3D_F64.matrixToQuaternion(sensorToTargetViaTransformRot, sensorToTargetViaTransformQuat);

                            double[] eulerBefore=new double[]{0,0,0};
                            ConvertRotation3D_F64.matrixToEuler(sensorToTargetViaTransformRot, EulerType.YXY,eulerBefore);
                            double[] eulerAfter = new double[] { eulerBefore[0], -1.0*eulerBefore[1], eulerBefore[2] };  // robot+Z+Y+Z = boof+Y-X+Y
                            ConvertRotation3D_F64.eulerToMatrix(EulerType.ZYZ, eulerAfter[0],   eulerAfter[1],      eulerAfter[2],    sensorToTargetViaTransformRot);
                            sensorToTargetViaTransform.setRotation(sensorToTargetViaTransformRot);
                            ConvertRotation3D_F64.matrixToQuaternion(sensorToTargetViaTransformRot, sensorToTargetViaTransformQuat);

                            //// TODO - timing here  c[camera_num]-f[frameprocessed]-detectionOrder_[iteration]-t[tagid]
                            Log.i(logTagTag,"after applying transformations");
                            int tag_id_reported = MARKER_OFFSET_INT+tag_id;

                            long reportDetectedFeatureStartTime = Calendar.getInstance().getTimeInMillis();
                            Log.i(logTagTag,"invoke() : after constructing grayImage in "+timeElapsed(algorithmStepStartTime)+"ms");
                            rosThingy.reportDetectedFeature(tag_id_reported,
                                    sensorToTargetViaTransform.getX(), sensorToTargetViaTransform.getY(), sensorToTargetViaTransform.getZ(),
                                    sensorToTargetViaTransformQuat.x,sensorToTargetViaTransformQuat.y,sensorToTargetViaTransformQuat.z,sensorToTargetViaTransformQuat.w);
                            Log.i(logTagTag, "ROS timing: rosThingy.reportDetectedFeature took "+timeElapsed(reportDetectedFeatureStartTime)+"ms");
                            System.out.println("3D Location: reporting tag_id "+tag_id_reported+" as : x = " + transBoofCV_TtoS.getX() + ", y = " + transBoofCV_TtoS.getY() + ", z = " + transBoofCV_TtoS.getZ());
                            System.out.println("3D Location: reporting tag_id "+tag_id_reported+" as : qx = " + quatBoofCV_TtoS.x + ", qy = " + quatBoofCV_TtoS.y + ", qz = " + quatBoofCV_TtoS.z + ", qw = " + quatBoofCV_TtoS.w);


                        /* Dev: part of robot visual model */
                            robotsDetected.put(singleDummyRobotId,robotFeatures);
                            Point2D_F64 locationPixel = new Point2D_F64();
                            detector.getImageLocation(detectionOrder_, locationPixel);        // pixel location in input image
                            boolean used=false;
                            if(Hardcoding.isPartOfRobotVisualModel(tag_id)) {
                                DetectedTag detectedTag = new DetectedTag(tag_id,sensorToTargetViaTransform,sensorToTargetViaTransformQuat);
                                robotFeatures.add(detectedTag);
                                used=true;
                                Log.i(logTagTag,"isPartOfRobotVisualModel TAG - tag_id "+tag_id+" - 2D Image Location = "+locationPixel);
                            }
                            if(Hardcoding.isALandmark(tag_id)) {
                                DetectedTag detectedTag = new DetectedTag(tag_id, sensorToTargetViaTransform, locationPixel);
                                landmarkFeatures.add(detectedTag);
                                if(used) {Log.w(logTagTag,"looks like tag_id"+tag_id+" is being used more than once");}
                                used=true;
                                Log.i(logTagTag, "isALandmark TAG - tag_id " + tag_id + " landmarkFeatures.size()=" + landmarkFeatures.size() + " - 2D Image Location = " + locationPixel);
                            }
                            if(Hardcoding.isAQuad(tag_id)) {
                                DetectedTag detectedTag = new DetectedTag(tag_id, sensorToTargetViaTransform, locationPixel);
                                quadFeatures.add(detectedTag);
                                if(used) {Log.w(logTagTag,"looks like tag_id"+tag_id+" is being used more than once");}
                                used=true;
                                Log.i(logTagTag, "isAQuad TAG - tag_id " + tag_id + " quadFeatures.size()=" + quadFeatures.size() + " - 2D Image Location = " + locationPixel);
                            }
                            if(!used){ // not part of something that we are looking for, so ignore
                                Log.i(logTagTag,"IGNORING TAG - not part of robot visual model - tag_id "+tag_id+" - 2D Image Location = "+locationPixel);
                                continue;
                            }
                        /* end Dev: part of robot visual model */

                            //// TODO - timing here  c[camera_num]-f[frameprocessed]-detectionOrder_[iteration]-t[tagid]
                            Log.i(logTagTag,"after detectedFeaturesClient.reportDetectedFeature");
                            updateLocationFromDetectedFeature(tag_id, logTagTag, sensorToTargetViaTransform, sensorToTargetViaTransformQuat);
    //                            variousUnusedAttemptsAtCoordinateSystemCorrection();

                        } else {  // 3D info not available for tag/marker
    //                            drawMarkerLocationOnDisplay_BoofCV(detector, detectionOrder_, FeatureModel.FEATURE_WITHOUT_3D_LOCATION);
                        }
                    }
                    if(quadFeatures.size() >=4 ) {
                        Log.i(logTag,"quadFeatures: enough quadFeatures found in image to estimate camera pose: quadFeatures.size()="+quadFeatures.size());
    //                        Se3_F64 estimateCameraPoseFrom3D2DPointMatches(CameraPinholeRadial cameraDistortionCoefficients, int numPointsToUse, double[] worldX, double[] worldY, double[] worldZ, double[] pixelsX, double[] pixelsY);
                        CameraPinholeRadial cameraIntrinsics = new CameraPinholeRadial(
                                focal_length_in_pixels_x, focal_length_in_pixels_y,
                                skew,
                                focal_midpoint_pixels_x, focal_midpoint_pixels_y,
                                imageWidthInt, imageHeightInt);


                        Se3_F64 cameraPose = updatePoseEstimate(cameraIntrinsics,quadFeatures);
                        int numElementsInRot = cameraPose.R.getNumElements();
                        Vector3D_F64 translation = cameraPose.T;
                        Log.i(TAG, "quadFeatures: cameraPose: " + /*" rotation numElements=" + numElementsInRot + ", rotation=" + cameraPose.R.toString() + */", translation =" + translation.toString());

                    } else {
                        Log.i(logTag,"quadFeatures: not enough quadFeatures found in image: quadFeatures.size()="+quadFeatures.size());
                    }

                } catch (IOException e) {
                    e.printStackTrace();
                } finally {
                    long timeNow = Calendar.getInstance().getTimeInMillis();
                    long timeElapsed = timeNow - procStartTime;
                    int fps = (int) (1.0/(timeElapsed/1000.0));
                    recordPerformance(procStartTime, fps);
                }
                executionEndTimeMs = Calendar.getInstance().getTimeInMillis();
                return new Long(0L);
            }
        }
    }

    class PerformanceMetric{
        private int fps, concurrentThreads;
        PerformanceMetric(int fps_, int concurrentThreads_) {
            fps=fps_;
            concurrentThreads=concurrentThreads_;
        }

        public int fps() { return fps; }

        public int concurrentThreads() { return concurrentThreads; }
    }


    Map<String, PerformanceMetric>countOfFpsAndThreads = Collections.synchronizedMap(new LinkedHashMap<String, PerformanceMetric>()  // convenient for the fixed-size
    {
        @Override
        protected boolean removeEldestEntry(Map.Entry<String, PerformanceMetric> eldest) {
            return this.size() > 50;
        }
    });


    Map<String, PerformanceMetric>logOfFpsAndThreads = Collections.synchronizedMap(new LinkedHashMap<String, PerformanceMetric>()  // convenient for the fixed-size
    {
        @Override
        protected boolean removeEldestEntry(Map.Entry<String, PerformanceMetric> eldest) {
            return this.size() > 200;
        }
    });

    ConcurrentLinkedQueue<GrayU8> unusedGrayImageQueue = new ConcurrentLinkedQueue<GrayU8>();   //  todo - persist in the application across onSleep/onResume for re-orientations

    private void finishedUsingGrayImage(GrayU8 image_) {
        Log.i("finishedUsingGrayImage","finishedUsingGrayImage");
        if(null!=unusedGrayImageQueue && null!=image_) {
            try {
                unusedGrayImageQueue.add(image_);
            } catch (Exception e) {
                Log.e(TAG, "Exception in unusedGrayImageQueue.add(image_): CONTINUING after this exception: ",e);
                e.printStackTrace();
            }
        }
    }
    private GrayU8 fetchAGrayImageToUse(int imageWidth_, int imageHeight_) {
        GrayU8 unusedImage;
        while(true) {
            try {
                unusedImage = unusedGrayImageQueue.remove();
            } catch (NoSuchElementException e) {                    // queue is empty, so make a new image
                Log.i("fetchAGrayImageToUse","queue is empty, so make a new image");
                return new GrayU8(imageWidth_, imageHeight_);
            }
            if(unusedImage.getWidth() == imageWidth_ && unusedImage.getHeight() == imageHeight_) {
                Log.i("fetchAGrayImageToUse","image is the right size");
                return unusedImage;
            } else if(unusedImage.getWidth() < imageWidth_ || unusedImage.getHeight() < imageHeight_) {     // too small: discard and continue
                Log.i("fetchAGrayImageToUse","image is too small: discard and continue");
                unusedImage = null;                                 // discard: setting to null to make intention plain.
            } else if(unusedImage.getWidth() >= imageWidth_ || unusedImage.getHeight() >= imageHeight_) {   // too large: can resize
                Log.i("fetchAGrayImageToUse","image is too large: can resize");
                unusedImage.reshape(imageWidth_,imageHeight_);      // resize "without declaring new memory."
                return unusedImage;
            }
        }
    }


    private static long timeElapsed(long startTime) {
        long timeNow;
        long timeElapsed;
        timeNow = Calendar.getInstance().getTimeInMillis();
        timeElapsed = timeNow - startTime;
        return timeElapsed;
    }

    /**
     * Compares two {@code Size}s based on their areas.
     */
    static class CompareSizesByArea implements Comparator<Size> {

        @Override
        public int compare(Size lhs, Size rhs) {
            // We cast here to ensure the multiplications won't overflow
            return Long.signum((long) lhs.getWidth() * lhs.getHeight() -
                    (long) rhs.getWidth() * rhs.getHeight());
        }

    }

    /**
     * Shows an error message dialog.
     */
    public static class ErrorDialog extends DialogFragment {

        private static final String ARG_MESSAGE = "message";

        public static ErrorDialog newInstance(String message) {
            ErrorDialog dialog = new ErrorDialog();
            Bundle args = new Bundle();
            args.putString(ARG_MESSAGE, message);
            dialog.setArguments(args);
            return dialog;
        }

        @Override
        public Dialog onCreateDialog(Bundle savedInstanceState) {
            final Activity activity = getActivity();
            return new AlertDialog.Builder(activity)
                    .setMessage(getArguments().getString(ARG_MESSAGE))
                    .setPositiveButton(android.R.string.ok, new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialogInterface, int i) {
                            activity.finish();
                        }
                    })
                    .create();
        }

    }

    /**
     * Shows OK/Cancel confirmation dialog about camera permission.
     */
    public static class ConfirmationDialog extends DialogFragment {

        @Override
        public Dialog onCreateDialog(Bundle savedInstanceState) {
            final Fragment parent = getParentFragment();
            return new AlertDialog.Builder(getActivity())
                    .setMessage(R.string.request_permission)
                    .setPositiveButton(android.R.string.ok, new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            FragmentCompat.requestPermissions(parent,
                                    new String[]{Manifest.permission.CAMERA},
                                    REQUEST_CAMERA_PERMISSION);
                        }
                    })
                    .setNegativeButton(android.R.string.cancel,
                            new DialogInterface.OnClickListener() {
                                @Override
                                public void onClick(DialogInterface dialog, int which) {
                                    Activity activity = parent.getActivity();
                                    if (activity != null) {
                                        activity.finish();
                                    }
                                }
                            })
                    .create();
        }
    }


    /*** reference back to Activity to get it to connect at appropriate point *********************************************/

    private RosThingy rosThingy;
    public void rosThingy(RosThingy rosThingy_) {  // TODO - circular reference - dispose of properly at appropriate point(s) in lifecycle
        this.rosThingy=rosThingy_;
    }

}
