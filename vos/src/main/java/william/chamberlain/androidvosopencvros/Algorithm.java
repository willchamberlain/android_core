package william.chamberlain.androidvosopencvros;

/**
 * Created by will on 25/09/17.
 */

public enum Algorithm {
        APRIL_TAGS_KAESS_36_H_11("AprilTags_Kaess_36h11"),
        BOOFCV_SQUARE_FIDUCIAL("boofcv square fiducial");

        private final String canonicalName;

        private Algorithm(String canonicalName_) {
            this.canonicalName = canonicalName_;
        }

        public String canonicalName() {
            return canonicalName;
        }

        public String toString() {
            return canonicalName;
        }

}
