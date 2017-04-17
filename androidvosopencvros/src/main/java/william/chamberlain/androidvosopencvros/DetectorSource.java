package william.chamberlain.androidvosopencvros;

import boofcv.abst.fiducial.FiducialDetector;
import boofcv.struct.image.GrayU8;

/**
 * Created by will on 13/04/17.
 */

interface DetectorSource {
    FiducialDetector<GrayU8> createDetector();
}
