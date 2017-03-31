package william.chamberlain.androidvosopencvros.device;

import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.os.Looper;

/**
 * Created by will on 22/03/17.
 */
public class ImuThread extends Thread {
    private final SensorManager sensorManager;
    private ImuSensorListenerPublisher imuSensorListenerPublisher;
    private Looper threadLooper;

    private final Sensor accelSensor;
    private final Sensor gyroSensor;
    private final Sensor quatSensor;
    private int sensorDelay;

    public ImuThread(SensorManager sensorManager, ImuSensorListenerPublisher imuSensorListenerPublisher, int sensorDelay_) {
        this.sensorManager = sensorManager;
        this.imuSensorListenerPublisher = imuSensorListenerPublisher;
        this.sensorDelay = sensorDelay_;
        this.accelSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        this.gyroSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        this.quatSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
    }


    public void run() {
        Looper.prepare();
        this.threadLooper = Looper.myLooper();
        this.sensorManager.registerListener(this.imuSensorListenerPublisher, this.accelSensor, this.sensorDelay);
        this.sensorManager.registerListener(this.imuSensorListenerPublisher, this.gyroSensor, this.sensorDelay);
        this.sensorManager.registerListener(this.imuSensorListenerPublisher, this.quatSensor, this.sensorDelay);
        Looper.loop();
    }


    public void shutdown() {
        this.sensorManager.unregisterListener(this.imuSensorListenerPublisher);
        if (this.threadLooper != null) {
            this.threadLooper.quit();
        }
    }
}
