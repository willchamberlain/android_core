package william.chamberlain.androidvosopencvros.device;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.SystemClock;

import org.ros.message.Time;
import org.ros.node.topic.Publisher;

import sensor_msgs.Imu;

/**
 * Created by will on 22/03/17.
 */
public class ImuSensorListenerForCallback implements SensorEventListener, ImuSensorListener {

    private boolean hasAccel;
    private boolean hasGyro;
    private boolean hasQuat;

    private long accelTime;
    private long gyroTime;
    private long quatTime;

    private ImuCallback callback;

    private Imu imu;

    public ImuSensorListenerForCallback( boolean hasAccel, boolean hasGyro, boolean hasQuat, ImuCallback callback_) {
        this.hasAccel = hasAccel;
        this.hasGyro = hasGyro;
        this.hasQuat = hasQuat;
        this.accelTime = 0;
        this.gyroTime = 0;
        this.quatTime = 0;
        this.callback = callback_;
    }

    //	@Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }

    //	@Override
    public void onSensorChanged(SensorEvent event) {
        boolean report = false;
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            float magnitude =
                    event.values[0]*event.values[0]
                    + event.values[1]*event.values[1]
                    + event.values[2]*event.values[2];
            if ( magnitude > 0.1) {
                report = true;
                this.accelTime = event.timestamp;
                System.out.println("ImuSensorListenerForCallback.onSensorChanged: TYPE_ACCELEROMETER triggered at "+magnitude);
            } else {
                System.out.println("ImuSensorListenerForCallback.onSensorChanged: TYPE_ACCELEROMETER below magnitude at "+magnitude);
            }
        } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            float magnitude =
                    event.values[1]*event.values[1]
                    + event.values[0]*event.values[0]
                    + event.values[2]*event.values[2];
                if ( magnitude > 0.1) {
                    report = true;
                    this.gyroTime = event.timestamp;
                    System.out.println("ImuSensorListenerForCallback.onSensorChanged: TYPE_GYROSCOPE triggered at "+magnitude);
                } else {
                    System.out.println("ImuSensorListenerForCallback.onSensorChanged: TYPE_GYROSCOPE below magnitude at "+magnitude);
                }
        } else if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
            float[] quaternion = new float[4];
            SensorManager.getQuaternionFromVector(quaternion, event.values);
            float magnitude = quaternion[0];
            if ( magnitude > 0.1) {
                report = true;
                this.quatTime = event.timestamp;
                System.out.println("ImuSensorListenerForCallback.onSensorChanged: TYPE_ROTATION_VECTOR triggered at "+magnitude);
            } else {
                System.out.println("ImuSensorListenerForCallback.onSensorChanged: TYPE_ROTATION_VECTOR below magnitude at "+magnitude);
            }
        }
        // Currently storing event times in case I filter them in the future.  Otherwise they are used to determine if all sensors have reported.
        if (   (this.accelTime != 0 && this.hasAccel)
            || (this.gyroTime  != 0 && this.hasGyro)
            || (this.quatTime  != 0 && this.hasQuat)) {
            // Convert event.timestamp (nanoseconds uptime) into system time, use that as the header stamp
            long time_delta_millis = System.currentTimeMillis() - SystemClock.uptimeMillis();
            this.imu.getHeader().setStamp(Time.fromMillis(time_delta_millis + event.timestamp / 1000000));
            this.imu.getHeader().setFrameId("/imu");// TODO Make parameter

            callback.imuData(imu);
            this.imu = new ImuData();

            // Reset times
            this.accelTime = 0;
            this.gyroTime = 0;
            this.quatTime = 0;
        }
//        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
//            this.imu.getLinearAcceleration().setX(event.values[0]);
//            this.imu.getLinearAcceleration().setY(event.values[1]);
//            this.imu.getLinearAcceleration().setZ(event.values[2]);
//
//            double[] tmpCov = {0, 0, 0, 0, 0, 0, 0, 0, 0};// TODO Make Parameter
//            this.imu.setLinearAccelerationCovariance(tmpCov);
//            this.accelTime = event.timestamp;
//        } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
//            this.imu.getAngularVelocity().setX(event.values[0]);
//            this.imu.getAngularVelocity().setY(event.values[1]);
//            this.imu.getAngularVelocity().setZ(event.values[2]);
//            double[] tmpCov = {0, 0, 0, 0, 0, 0, 0, 0, 0};// TODO Make Parameter
//            this.imu.setAngularVelocityCovariance(tmpCov);
//            this.gyroTime = event.timestamp;
//        } else if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
//            float[] quaternion = new float[4];
//            SensorManager.getQuaternionFromVector(quaternion, event.values);
//            this.imu.getOrientationQuaternionXyzw().setW(quaternion[0]);
//            this.imu.getOrientationQuaternionXyzw().setX(quaternion[1]);
//            this.imu.getOrientationQuaternionXyzw().setY(quaternion[2]);
//            this.imu.getOrientationQuaternionXyzw().setZ(quaternion[3]);
//            double[] tmpCov = {0, 0, 0, 0, 0, 0, 0, 0, 0};// TODO Make Parameter
//            this.imu.setOrientationCovariance(tmpCov);
//            this.quatTime = event.timestamp;
//        }
//
//        // Currently storing event times in case I filter them in the future.  Otherwise they are used to determine if all sensors have reported.
//        if ((this.accelTime != 0 || !this.hasAccel) &&
//                (this.gyroTime != 0 || !this.hasGyro) &&
//                (this.quatTime != 0 || !this.hasQuat)) {
//            // Convert event.timestamp (nanoseconds uptime) into system time, use that as the header stamp
//            long time_delta_millis = System.currentTimeMillis() - SystemClock.uptimeMillis();
//            this.imu.getHeader().setStamp(Time.fromMillis(time_delta_millis + event.timestamp / 1000000));
//            this.imu.getHeader().setFrameId("/imu");// TODO Make parameter
//
//            callback.imuData(imu);
//            this.imu = new ImuData();
//
//            // Reset times
//            this.accelTime = 0;
//            this.gyroTime = 0;
//            this.quatTime = 0;
//        }
    }

}
