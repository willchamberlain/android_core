package william.chamberlain.androidvosopencvros.device;

import sensor_msgs.Imu;

/**
 * Created by will on 22/03/17.
 */

public interface ImuCallback {
    public void imuData(Imu imu);
}
