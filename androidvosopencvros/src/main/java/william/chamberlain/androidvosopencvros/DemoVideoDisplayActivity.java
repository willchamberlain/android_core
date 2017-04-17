package william.chamberlain.androidvosopencvros;

import android.hardware.Camera;
import android.os.Bundle;

import boofcv.android.gui.VideoDisplayActivity;

/**
 * Activity for displaying video results.
 *
 * @author Peter Abeles
 */
public class DemoVideoDisplayActivity extends VideoDisplayActivity {

	public static ApplicationPreferences preference;

	public DemoVideoDisplayActivity() {
	}

	public DemoVideoDisplayActivity(boolean hidePreview) {
		super(hidePreview);
	}

	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

		// SEE  /mnt/nixbig/downloads/boofcv_will/BoofAndroidDemo/app/src/main/java/org/boofcv/android/DemoMain.java
		setShowFPS(preference.showFps);
	}

	@Override
	protected Camera openConfigureCamera( Camera.CameraInfo info ) {								// set up the video/image input along with the UI - can only get video as it comes though the preview
		Camera mCamera = Camera.open(preference.cameraId);
		Camera.getCameraInfo(preference.cameraId,info);

		Camera.Parameters param = mCamera.getParameters();
		Camera.Size sizePreview = param.getSupportedPreviewSizes().get(preference.preview);
		param.setPreviewSize(sizePreview.width,sizePreview.height);
		Camera.Size sizePicture = param.getSupportedPictureSizes().get(preference.picture);
		param.setPictureSize(sizePicture.width, sizePicture.height);
		mCamera.setParameters(param);

		return mCamera;
	}
}