package william.chamberlain.androidvosopencvros;

import boofcv.struct.calib.CameraPinholeRadial;

/**
 * @author Peter Abeles
 */
public class ApplicationPreferences {
	public int cameraId;					// ID number of the camera to use, out of the list of cameras in CameraUtils.specs
	public int preview = 5;
	public int picture;
	public boolean showFps = true;
	public boolean showInputImage = true;
    public boolean drawText = true;
	public CameraPinholeRadial cameraIntrinsics;
}
