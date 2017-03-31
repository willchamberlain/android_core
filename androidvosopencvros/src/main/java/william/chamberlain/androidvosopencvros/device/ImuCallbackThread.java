package william.chamberlain.androidvosopencvros.device;

import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.os.Looper;

/**
 * Created by will on 22/03/17.
 */
public class ImuCallbackThread extends Thread {
    private final SensorManager sensorManager;
    private ImuSensorListenerForCallback imuSensorListenerForCallback;
    private Looper threadLooper;

    private final Sensor accelSensor;
    private final Sensor gyroSensor;
    private final Sensor quatSensor;
    private int sensorDelay;

    public ImuCallbackThread(SensorManager sensorManager, ImuSensorListenerForCallback imuSensorListenerForCallback_, int sensorDelay_) {
        this.sensorManager = sensorManager;
        this.imuSensorListenerForCallback = imuSensorListenerForCallback_;
        this.sensorDelay = sensorDelay_;
        this.accelSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        this.gyroSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        this.quatSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
    }


    public void run() {
        Looper.prepare();
        this.threadLooper = Looper.myLooper();
        this.sensorManager.registerListener(this.imuSensorListenerForCallback, this.accelSensor, this.sensorDelay);
        this.sensorManager.registerListener(this.imuSensorListenerForCallback, this.gyroSensor, this.sensorDelay);
        this.sensorManager.registerListener(this.imuSensorListenerForCallback, this.quatSensor, this.sensorDelay);
        Looper.loop();
    }


    public void shutdown() {
        this.sensorManager.unregisterListener(this.imuSensorListenerForCallback);
        if (this.threadLooper != null) {
            this.threadLooper.quit();
        }
    }
}
