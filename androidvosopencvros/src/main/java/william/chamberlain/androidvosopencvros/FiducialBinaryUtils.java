package william.chamberlain.androidvosopencvros;

import android.support.annotation.NonNull;

import boofcv.factory.fiducial.ConfigFiducialBinary;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.factory.filter.binary.ConfigThreshold;
import boofcv.factory.filter.binary.ThresholdType;
import boofcv.struct.image.GrayU8;

/**
 * Created by will on 12/04/17.
 */

public class FiducialBinaryUtils {

    @NonNull
    static boofcv.abst.fiducial.FiducialDetector<GrayU8> createGrayU8FiducialDetector(int binaryThreshold, Object lock, boolean robust) {
        boofcv.abst.fiducial.FiducialDetector<GrayU8> detector;
        ConfigFiducialBinary config = new ConfigFiducialBinary(0.162);

        synchronized (lock) {
            ConfigThreshold configThreshold;
            if (robust) {
                configThreshold = ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 6);
            } else {
                configThreshold = ConfigThreshold.fixed(binaryThreshold);
            }
            detector = FactoryFiducial.squareBinary(config, configThreshold, GrayU8.class);
        }

        return detector;
    }
}
